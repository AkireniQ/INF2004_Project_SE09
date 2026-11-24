#include "vehicle_core.h"

bool vc_core_detouring(const VC_MODEL *model)
{
    return model->state == VC_AVOIDING || model->state == VC_RECOVER_LINE;
}

bool vc_core_running(const VC_MODEL *model)
{
    return model->state == VC_FOLLOW_LINE || vc_core_detouring(model);
}

bool vc_core_needs_line(const VC_MODEL *model)
{
    if (model->state == VC_FOLLOW_LINE) return true;
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
    model->await_forward_range = false;
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

    /* Forward ranging resumes only once the tape is confirmed. Wait for a
       fresh reading at that point before sending line-follow velocity. */
    if (m->await_forward_range) {
        if (!in->avoidance.range_valid) return;
        m->await_forward_range = false;
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
        bool confirming_line = m->avoidance.phase == S5_CHECK_LINE;
        out->maneuver = s5_avoid_step(&m->avoidance, &in->avoidance);
        S5_PHASE phase = m->avoidance.phase;
        if (phase == S5_FAILED) { halt(m, in, VC_FAULT, VC_REASON_OBSTACLE); return; }
        if (phase == S5_COMPLETE) {
            m->state = VC_FOLLOW_LINE;
            m->reason = VC_REASON_NONE;
            m->await_forward_range = confirming_line;
            return; /* Hand off using the next fresh line-follow command. */
        }
        m->state = phase == S5_FIND_LINE || phase == S5_CENTER_LINE ||
                   phase == S5_ALIGN_LINE || phase == S5_CHECK_LINE ?
                   VC_RECOVER_LINE : VC_AVOIDING;
        out->stop = out->maneuver.action == S5_STOP;
        return;
    }

    m->state = VC_FOLLOW_LINE;
    m->reason = VC_REASON_NONE;
    bool approach = in->avoidance.range_valid && in->avoidance.range_mm <= S5_SLOW_RANGE_MM;
    out->stop = false;
    out->left_rpm = approach && in->left_rpm > S5_APPROACH_RPM ? S5_APPROACH_RPM : in->left_rpm;
    out->right_rpm = approach && in->right_rpm > S5_APPROACH_RPM ? S5_APPROACH_RPM : in->right_rpm;
}
