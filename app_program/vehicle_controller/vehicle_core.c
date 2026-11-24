#include "vehicle_core.h"

/* Includes the motion owner's 200 ms direction-settling guard. */
#define VC_JUNCTION_SETTLE_MS 230u
#define VC_JUNCTION_TIMEOUT_MS 8000u

bool vc_core_detouring(const VC_MODEL *model)
{
    return model->state == VC_AVOIDING || model->state == VC_RECOVER_LINE;
}

bool vc_core_running(const VC_MODEL *model)
{
    return model->state == VC_FOLLOW_LINE || model->state == VC_TURNING ||
           vc_core_detouring(model);
}

bool vc_core_needs_line(const VC_MODEL *model)
{
    if (model->state == VC_FOLLOW_LINE || model->state == VC_TURNING) return true;
    S5_PHASE phase = model->avoidance.phase;
    return vc_core_detouring(model) &&
           (phase == S5_BACK_UP || phase == S5_FIND_LINE || phase == S5_CHECK_LINE);
}

VC_START_BLOCK vc_core_start_block(const VC_MODEL *model, const VC_INPUT *in)
{
    if (in->stop || in->reset || (!vc_core_running(model) && in->motor_busy))
        return VC_START_STOP_PENDING;
    if (in->motor_faults) return VC_START_MOTOR;
    if (!in->link_ok) return VC_START_LINK;
    if (!in->ready) return VC_START_STARTUP;
    if (!in->line_valid) return VC_START_NO_LINE;
    if (in->line_age_ms > VC_LINE_MAX_AGE_MS) return VC_START_OLD_LINE;
    if (in->line_stop) return VC_START_LINE_STOP;
    if ((model->reposition_required || model->avoidance.phase == S5_FAILED) &&
        ((!in->avoidance.line_hit && !in->avoidance.line_center_seen) || in->avoidance.line_junction))
        return VC_START_OBSTACLE;
    return VC_START_READY;
}

static void halt(VC_MODEL *model, const VC_INPUT *in, VC_STATE state, VC_REASON reason)
{
    if (model->state == VC_TURNING) model->reposition_required = true;
    if (vc_core_detouring(model)) {
        S5_PHASE phase = model->avoidance.phase;
        if (phase == S5_PROFILE_PATH || phase == S5_WAIT_PATH ||
            ((phase == S5_TURN_OUT || phase == S5_BACK_UP) &&
             in->avoidance.motion == S5_MOTION_NONE)) {
            model->avoidance = (S5_AVOID){0};
        } else {
            S5_FAILURE failure = reason == VC_REASON_STOP ? S5_FAIL_STOP :
                                 reason == VC_REASON_MOTOR ? S5_FAIL_MOTION : S5_FAIL_INTERLOCK;
            s5_avoid_abort(&model->avoidance, failure, in->avoidance.now);
        }
    }
    model->state = state;
    model->reason = reason;
}

static void junction_phase(VC_MODEL *m, VC_JUNCTION_PHASE phase, const VC_INPUT *in)
{
    m->junction.phase = phase;
    m->junction.phase_at = in->avoidance.now;
    m->junction.line_at = in->avoidance.line_at;
    m->junction.dark_samples = m->junction.light_samples = 0;
}

static void junction_step(VC_MODEL *m, const VC_INPUT *in, VC_OUTPUT *out)
{
    const S5_AVOID_INPUT *sensors = &in->avoidance;
    S5_MOTION motion = sensors->motion;
    unsigned elapsed = sensors->now - m->junction.phase_at;
    bool first_pivot = m->junction.phase == VC_JUNCTION_PIVOT;
    bool right = first_pivot ? m->junction.right : !m->junction.right;

    if (motion == S5_MOTION_FAILED) {
        halt(m, in, VC_FAULT, VC_REASON_MOTOR);
        return;
    }
    if (elapsed > VC_JUNCTION_TIMEOUT_MS) {
        halt(m, in, VC_FAULT, VC_REASON_BARCODE_LINE);
        return;
    }

    if (m->junction.phase == VC_JUNCTION_SETTLE) {
        if (sensors->stopped && elapsed >= VC_JUNCTION_SETTLE_MS && !sensors->line_junction) {
            m->state = VC_FOLLOW_LINE;
        }
        return;
    }

    if (m->junction.phase == VC_JUNCTION_ADVANCE) {
        if (motion == S5_MOTION_DONE) {
            junction_phase(m, VC_JUNCTION_PIVOT, in);
            out->stop = false;
        } else if (motion != S5_MOTION_NONE) {
            out->stop = false; /* Keep the tracked move running. */
        } else if (sensors->stopped && elapsed >= VC_JUNCTION_SETTLE_MS) {
            /* Both IR sensors first see the near edge of the crossing tape. */
            out->maneuver = (S5_AVOID_OUTPUT){
                .action = S5_ADVANCE,
                .distance_mm = S5_IR_TO_AXLE_MM + S5_TAPE_WIDTH_MM / 2u,
            };
            out->stop = false;
        }
        return;
    }

    if (in->pivot_in_window && sensors->line_at != m->junction.line_at) {
        m->junction.line_at = sensors->line_at;
        bool leading = right ? sensors->right_on_tape : sensors->left_on_tape;
        bool trailing = right ? sensors->left_on_tape : sensors->right_on_tape;
        if (leading && !trailing) {
            if (m->junction.dark_samples < 2u) ++m->junction.dark_samples;
        } else if (m->junction.dark_samples < 2u || trailing) {
            m->junction.dark_samples = 0;
        }
        if (m->junction.dark_samples == 2u && !leading && !trailing) {
            if (++m->junction.light_samples == 2u) {
                junction_phase(m, VC_JUNCTION_SETTLE, in);
                return; /* The tape is now between the tracking sensors. */
            }
        } else {
            m->junction.light_samples = 0;
        }
    }

    if (motion == S5_MOTION_DONE) {
        if (first_pivot) {
            junction_phase(m, VC_JUNCTION_SWEEP, in);
            out->stop = false;
        } else {
            halt(m, in, VC_FAULT, VC_REASON_BARCODE_LINE);
        }
    } else if (motion != S5_MOTION_NONE) {
        out->stop = false;
    } else if (sensors->stopped && elapsed >= VC_JUNCTION_SETTLE_MS) {
        unsigned nominal = m->junction.uturn ? 180u : 90u;
        out->pivot_degrees = first_pivot ? nominal + VC_JUNCTION_SEARCH_DEG :
                                          2u * VC_JUNCTION_SEARCH_DEG;
        out->pivot_right = right;
        out->stop = false;
    }
}

void vc_core_step(VC_MODEL *m, const VC_INPUT *in, VC_OUTPUT *out)
{
    *out = (VC_OUTPUT){.stop = true};
    if (in->stop) { halt(m, in, VC_IDLE, VC_REASON_STOP); return; }
    if (in->motor_faults || in->drive_failed) { halt(m, in, VC_FAULT, VC_REASON_MOTOR); return; }
    if (in->reset) return;
    if (!vc_core_running(m)) {
        if (!in->start || vc_core_start_block(m, in) != VC_START_READY) return;
        m->state = VC_FOLLOW_LINE;
        m->reposition_required = false;
        m->avoidance = (S5_AVOID){0};
    }

    /* Once START is accepted, navigation uses local sensors. */
    if (!in->ready) { halt(m, in, VC_FAULT, VC_REASON_MOTOR); return; }
    if (vc_core_needs_line(m) && (!in->line_valid || in->line_age_ms > VC_LINE_MAX_AGE_MS)) {
        S5_PHASE phase = m->avoidance.phase;
        /* A completed encoder leg can wait at rest for the next valid IR
           sample. Once a tape search starts, stale data still stops it. */
        if (vc_core_detouring(m) && (phase == S5_FIND_LINE || phase == S5_CHECK_LINE) &&
            in->avoidance.motion == S5_MOTION_NONE &&
            in->avoidance.now - m->avoidance.phase_at < VC_LINE_REACQUIRE_WAIT_MS) return;
        halt(m, in, VC_FAULT, VC_REASON_STALE_LINE);
        return;
    }
    if (in->line_stop) { halt(m, in, VC_IDLE, VC_REASON_LINE_STOP); return; }

    if (m->state == VC_TURNING) {
        if (in->obstacle_blocked && in->avoidance.range_valid) {
            halt(m, in, VC_FAULT, VC_REASON_OBSTACLE);
        } else {
            junction_step(m, in, out);
        }
        return;
    }

    if (!vc_core_detouring(m) && in->obstacle_blocked) {
        /* An echo dropout alone cannot start avoidance. Keep the wheels
           stopped if a close obstacle was seen and its reading has gone stale. */
        if (!in->avoidance.range_valid) return;
        s5_avoid_begin(&m->avoidance, in->avoidance.now);
        m->avoidance.reverse_available = in->avoidance.can_retrace;
        m->avoidance.reverse_long = in->avoidance.can_retrace_far;
        m->state = VC_AVOIDING;
        m->reason = VC_REASON_OBSTACLE;
        return; /* Stop the line-follow velocity before starting the scan. */
    }
    if (vc_core_detouring(m)) {
        out->maneuver = s5_avoid_step(&m->avoidance, &in->avoidance);
        S5_PHASE phase = m->avoidance.phase;
        if (phase == S5_FAILED) { halt(m, in, VC_FAULT, VC_REASON_OBSTACLE); return; }
        if (phase == S5_COMPLETE) {
            m->state = VC_FOLLOW_LINE;
            m->reason = VC_REASON_NONE;
            return; /* Resume from fresh IR data on the next control cycle. */
        }
        m->state = phase == S5_FIND_LINE || phase == S5_CENTER_LINE ||
                   phase == S5_ALIGN_LINE || phase == S5_CHECK_LINE ?
                   VC_RECOVER_LINE : VC_AVOIDING;
        out->stop = out->maneuver.action == S5_STOP;
        return;
    }

    if (in->barcode_turn) {
        m->state = VC_TURNING;
        m->junction.right = in->barcode_turn != VC_NAV_LEFT;
        m->junction.uturn = in->barcode_turn == VC_NAV_UTURN;
        junction_phase(m, VC_JUNCTION_ADVANCE, in);
        return;
    }

    m->state = VC_FOLLOW_LINE;
    m->reason = VC_REASON_NONE;
    bool approach = in->avoidance.range_valid && in->avoidance.range_mm <= S5_SLOW_RANGE_MM;
    out->stop = false;
    out->left_rpm = approach && in->left_rpm > S5_APPROACH_RPM ? S5_APPROACH_RPM : in->left_rpm;
    out->right_rpm = approach && in->right_rpm > S5_APPROACH_RPM ? S5_APPROACH_RPM : in->right_rpm;
}
