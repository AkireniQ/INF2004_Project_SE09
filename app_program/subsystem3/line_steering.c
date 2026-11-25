#include "line_steering.h"

#define WIDE_MARKING_DARKNESS 800
#define STRAIGHT_ENTER 80
#define STRAIGHT_RELEASE 50
#define CURVE_DARKNESS 350
#define CURVE_IMBALANCE 315
#define CURVE_STRENGTH_AT_ENTRY 8u
#define CURVE_HOLD_SAMPLES 14
#define CURVE_TIGHT_SAMPLES 24
#define CURVE_RECENTER_SAMPLES 2
#define CURVE_EXIT_CONFIRM_SAMPLES 24
#define SHORT_GAP_SAMPLES 10
#define NEUTRAL_SAMPLES 4
#define CURVE_EXIT_NEUTRAL_SAMPLES 4


static int absolute(int value)
{
    return value < 0 ? -value : value;
}

static void finish_curve(LF_STATE *state)
{
    if (state->sustained_curve) { state->settle_samples = S3_CURVE_EXIT_SAMPLES; }
    state->candidate_side = 0;
    state->candidate_samples = 0;
    state->curve_side = 0;
    state->strong_samples = 0;
    state->light_samples = 0;
    state->centered_samples = 0;
    state->sustained_curve = false;
    state->filtered_error = 0;
}

static void confirm_curve(LF_STATE *state, int strong_side)
{
    if (strong_side == 0 || state->candidate_side != strong_side) {
        state->candidate_side = (int8_t)strong_side;
        state->candidate_samples = 0;
    }
    /* An edge touch after a bend or during barcode capture can be a brief
       drift. Keep the mild trim until the same edge persists. */
    unsigned confirm_samples = state->settle_samples || state->reading_barcode ?
                               CURVE_EXIT_CONFIRM_SAMPLES : S3_CURVE_CONFIRM_SAMPLES;
    if (strong_side != 0 && ++state->candidate_samples >= confirm_samples) {
        state->curve_side = (int8_t)strong_side;
        state->strong_samples = CURVE_STRENGTH_AT_ENTRY;
        state->settle_samples = 0;
    }
}

static int track_demand(LF_STATE *state, int error, bool light)
{
    if (light) {
        state->filtered_error = 0;
        return 0;
    }
    int previous = state->filtered_error;
    state->filtered_error += (int16_t)((error - previous) / 2);
    int magnitude = absolute(state->filtered_error);
    int side = state->filtered_error < 0 ? -1 : 1;
    bool already_steering = state->correction_rpm * side > 0;
    int trend = (state->filtered_error - previous) * side;
    if (already_steering && trend < 0) {
        /* Release the trim early when the edge is already coming back. */
        magnitude += 6 * trend;
    }
    int enter = state->settle_samples ? 2 * STRAIGHT_ENTER : STRAIGHT_ENTER;
    int threshold = already_steering ? STRAIGHT_RELEASE : enter;
    if (magnitude < threshold) { return 0; }
    if (!already_steering &&
        (previous * side < enter || error * side < STRAIGHT_RELEASE)) {
        return 0;
    }
    return side * S3_CORRECTION_STEP_RPM;
}

static int curve_demand(LF_STATE *state, int error, int strong_side, bool light)
{
    int side = state->curve_side;
    if (light) {
        state->centered_samples = 0;
        if (state->light_samples < S3_BLIND_CURVE_LIMIT) { ++state->light_samples; }
        if (state->strong_samples < CURVE_HOLD_SAMPLES) {
            /* A short edge touch can finish with both sensors on white. */
            if (state->light_samples >= SHORT_GAP_SAMPLES) {
                finish_curve(state);
                return 0;
            }
            return side * 4;
        }
        /* A sustained bend followed by an edge loss calls for a tight arc
         * toward the last seen edge. The caller stops if it stays blind. */
        return side * (state->light_samples < SHORT_GAP_SAMPLES ? 8 : S3_CRUISE_RPM);
    }

    state->light_samples = 0;
    if (side * error < 120) {
        if (++state->centered_samples >= CURVE_RECENTER_SAMPLES) {
            finish_curve(state);

            /* Stop carrying the old turn command into straight tracking. */
            state->correction_rpm = 0;
            state->slew_hold = false;
        }
        return 0;
    }
    state->centered_samples = 0;
    if (strong_side != side) {
        if (state->strong_samples > 0) { --state->strong_samples; }
        return side * 4;
    }
    if (state->strong_samples < CURVE_TIGHT_SAMPLES) { ++state->strong_samples; }
    if (state->strong_samples >= CURVE_HOLD_SAMPLES) { state->sustained_curve = true; }
    if (state->strong_samples >= CURVE_TIGHT_SAMPLES) { return side * S3_CRUISE_RPM; }
    return side * (state->strong_samples >= CURVE_HOLD_SAMPLES ? 8 : 6);
}

void line_decide(int left, int center, int right, LF_STATE *state, LF_CMD *command)
{
    int demand = 0;
    int error = right - left;
    int peak = left > right ? left : right;
    bool light = peak < S3_EDGE_DARKNESS;
    int strong_side = 0;
    if (state->settle_samples > 0u) { --state->settle_samples; }

    /* GP27 belongs to barcode sensing. */
    (void)center;
    if (peak >= CURVE_DARKNESS && absolute(error) >= CURVE_IMBALANCE) {
        strong_side = error < 0 ? -1 : 1;
    }
    if (left >= WIDE_MARKING_DARKNESS && right >= WIDE_MARKING_DARKNESS) {
        finish_curve(state);
    } else {
        if (state->curve_side == 0) { confirm_curve(state, strong_side); }
        demand = state->curve_side != 0 ? curve_demand(state, error, strong_side, light)
                                        : track_demand(state, error, light);
    }

    if (state->light_samples >= S3_BLIND_CURVE_LIMIT) {
        *command = (LF_CMD){.stop = true};
        return;
    }

    unsigned neutral_required = state->settle_samples || state->reading_barcode ?
                                 CURVE_EXIT_NEUTRAL_SAMPLES : NEUTRAL_SAMPLES;
    if (demand * state->last_steer_side < 0 && state->neutral_samples < neutral_required) {
        demand = 0;
    }
    if (state->slew_hold) {
        state->slew_hold = false;
    } else if (demand != state->correction_rpm) {
        state->correction_rpm += demand > state->correction_rpm
                               ? S3_CORRECTION_STEP_RPM : -S3_CORRECTION_STEP_RPM;
        state->slew_hold = true;
    }
    if (state->correction_rpm != 0) {
        state->last_steer_side = state->correction_rpm < 0 ? -1 : 1;
        state->neutral_samples = 0;
    } else if (state->neutral_samples < CURVE_EXIT_NEUTRAL_SAMPLES) {
        ++state->neutral_samples;
    }
    int left_rpm = S3_CRUISE_RPM + state->correction_rpm;
    int right_rpm = S3_CRUISE_RPM - state->correction_rpm;
    /* On the straight, use half the previous wheel-speed difference.
     * The turn targets still use the full range. */
    int outer_limit = state->curve_side == 0 &&
                      absolute(state->correction_rpm) <= S3_CORRECTION_STEP_RPM
                      ? S3_CRUISE_RPM : S3_MAX_OUTER_RPM;
    *command = (LF_CMD){
        .left = left_rpm > outer_limit ? outer_limit : left_rpm,
        .right = right_rpm > outer_limit ? outer_limit : right_rpm
    };
}
