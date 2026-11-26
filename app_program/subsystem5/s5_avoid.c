#include "s5_avoid.h"
#include "../subsystem2/motion_config.h"
#include "../subsystem3/line_steering.h"

#define FRONT_MARGIN_MM 15u
#define MAX_LATERAL_OFFSET_MM 400u
#define MOTION_SETTLE_MS 260u
#define SCAN_TIMEOUT_MS 15000u
#define RESCAN_INTERVAL_MS 1000u
#define RESCAN_RANGE_CHANGE_MM 30u
#define PHASE_TIMEOUT_MS 9000u
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3141u / 1000u)
#define TAPE_HALF_WIDTH_MM ((S5_TAPE_WIDTH_MM + 1u) / 2u)
#define REJOIN_START_CLEARANCE_MM 8u
#define MIN_REJOIN_OFFSET_MM (S5_IR_TO_AXLE_MM + TAPE_HALF_WIDTH_MM + REJOIN_START_CLEARANCE_MM)
#define EDGE_OFFSET_DEADBAND_MM 20u
#define LINE_ALIGN_TOLERANCE_MM 3

_Static_assert(S5_PIVOT_DEG == 90u, "The straight bypass legs assume right-angle pivots");
_Static_assert(S5_OUTWARD_CLEARANCE_BEARING_DEG == 60u,
               "Outward clearance projects the 60-degree sonar bearing");
_Static_assert(S5_PASS_DEPTH_PERCENT > 0u && S5_PASS_DEPTH_PERCENT <= 100u,
               "Estimated depth must be a percentage of the measured width");
_Static_assert(S5_FOLLOW_CLEAR_MM + S5_PASS_UNCERTAIN_DEPTH_MM + S5_PASS_BODY_ALLOWANCE_MM <= S5_MAX_PASS_MM,
               "The conservative default depth must fit the bypass budget");
_Static_assert(S5_FOLLOW_CLEAR_MM + S5_LONG_REVERSE_MM <= S5_MAX_PROJECTED_FRONT_MM,
               "The retry window must fit the sonar projection");
_Static_assert(S5_MAX_PASS_MM < 1024u, "Total pass distance must fit the 10-bit fields");
_Static_assert(S5_CHASSIS_WIDTH_MM >= EFFECTIVE_TRACK_MM, "Include the outer wheel edges in chassis width");
_Static_assert(MAX_LATERAL_OFFSET_MM <= 511u, "Lateral offset must fit in 9 bits");
_Static_assert(S5_IR_TO_AXLE_MM > 0u && S5_OUTER_IR_SPACING_MM > S5_TAPE_WIDTH_MM &&
                   MIN_REJOIN_OFFSET_MM <= MAX_LATERAL_OFFSET_MM,
               "The outer sensors must straddle the tape and fit within the bypass limit");
_Static_assert(MAX_LATERAL_OFFSET_MM +
                   S5_SEARCH_MARGIN_MM + S5_SEARCH_EXTENSION_MM <= S5_MAX_LEG_MM && S5_MAX_LEG_MM <= 1023u,
               "Tape search distance must fit the bounded encoder command");

static bool at_or_after(uint32_t time, uint32_t reference)
{
    return (uint32_t)(time - reference) < UINT32_C(0x80000000);
}

static S5_AVOID_OUTPUT go(S5_AVOID *state, S5_PHASE phase, uint32_t now)
{
    state->phase = phase;
    state->phase_at = now;
    state->line_samples = 0;
    state->last_line_at = now;
    state->last_range_at = now;
    state->range_seen = false;
    state->search_exhausted = false;
    return (S5_AVOID_OUTPUT){.action = S5_WAIT};
}

static bool confirmed_forward(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    if (in->range_mm == S5_OPEN_RANGE) return true; /* The range gate already confirmed two pings. */
    if (in->range_at == state->last_range_at) return false;
    state->last_range_at = in->range_at;
    bool confirmed = state->range_seen;
    state->range_seen = true;
    return confirmed;
}

void s5_avoid_abort(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    if (state->phase == S5_FAILED) return;
    state->failure = reason;
    state->failed_phase = state->phase;
    go(state, S5_FAILED, now);
}

static S5_AVOID_OUTPUT fail(S5_AVOID *state, S5_FAILURE reason, uint32_t now)
{
    s5_avoid_abort(state, reason, now);
    return (S5_AVOID_OUTPUT){.action = S5_STOP};
}

void s5_avoid_begin(S5_AVOID *state, uint32_t now)
{
    *state = (S5_AVOID){.phase = S5_PROFILE_PATH, .phase_at = now};
}

static unsigned reentry_detection_mm(unsigned offset_mm)
{
    unsigned margin = S5_IR_TO_AXLE_MM + TAPE_HALF_WIDTH_MM + S5_SEARCH_MARGIN_MM;
    return offset_mm > margin ? offset_mm - margin : 0u;
}

bool s5_avoid_executing(S5_PHASE phase)
{
    return (phase >= S5_TURN_OUT && phase <= S5_ALIGN_LINE) || phase == S5_CHECK_LINE;
}

unsigned s5_planned_pass_mm(const S5_PROFILE *profile, unsigned forward_mm)
{
    /* Estimate the shallow box depth separately from the chassis allowance. */
    unsigned depth = S5_PASS_UNCERTAIN_DEPTH_MM;
    if (profile->width_mm && profile->width_mm != S5_UNKNOWN_WIDTH) {
        depth = (profile->width_mm * S5_PASS_DEPTH_PERCENT + 99u) / 100u;
        if (profile->width_status != S5_WIDTH_OK) {
            depth += S5_UNCERTAIN_EDGE_MM;
            if (depth < S5_PASS_UNCERTAIN_DEPTH_MM) depth = S5_PASS_UNCERTAIN_DEPTH_MM;
        }
    }
    unsigned gap = profile->nearest_mm;
    if (s5_range_measured(forward_mm) && forward_mm > gap) gap = forward_mm;
    return gap + depth + S5_PASS_BODY_ALLOWANCE_MM;
}

static unsigned bypass_offset(int lateral_extent, bool edge_valid)
{
    int offset = lateral_extent + (int)S5_SIDE_ALLOWANCE_MM;
    if (!edge_valid) {
        /* A partial profile retains its last observed edge and an uncertainty gap. */
        unsigned extent = lateral_extent > 0 ? (unsigned)lateral_extent : 0u;
        offset = (int)(S5_SIDE_ALLOWANCE_MM + extent + S5_UNCERTAIN_EDGE_MM);
    }
    /* Leave both IR sensors on the approach side until the straight tape search. */
    return offset > (int)MIN_REJOIN_OFFSET_MM ? (unsigned)offset : MIN_REJOIN_OFFSET_MM;
}

static unsigned side_range(unsigned heading, unsigned background, unsigned required)
{
    /* A near return at 45 degrees takes priority over a distant reflection
       just past an obstacle edge. */
    if (heading <= S5_MANEUVER_STOP_MM) return heading;
    if (s5_range_available(heading) && heading >= required) return heading;
    if (s5_range_available(background)) return background;
    return heading;
}

static unsigned required_side_range(unsigned offset)
{
    return offset + FRONT_MARGIN_MM + S5_MANEUVER_STOP_MM;
}

static bool outward_probe_allowed(unsigned heading, unsigned offset)
{
    /* A known obstruction on departure takes priority over the outward echo. */
    return !s5_range_available(heading) || heading >= offset + S5_TURN_CLEARANCE_MM;
}

static bool outward_observed(unsigned range)
{
    return range != 0 && s5_range_available(range);
}

static bool outward_route_fits(unsigned heading, unsigned outward, unsigned offset)
{
    /* The 60-degree reading must project beyond the chassis and side margin. */
    unsigned lateral_room = offset + (S5_CHASSIS_WIDTH_MM + 1u) / 2u + FRONT_MARGIN_MM;
    return outward_probe_allowed(heading, offset) && outward_observed(outward) &&
           outward >= (lateral_room * 1000u + 865u) / 866u;
}

unsigned s5_scan_range_limit(const S5_AVOID *state)
{
    unsigned extra = state->scan_after_retrace ?
                     (state->reverse_long ? S5_LONG_REVERSE_MM : S5_REVERSE_MM) : 0u;
    return S5_FOLLOW_CLEAR_MM + extra;
}

S5_BYPASS s5_plan_bypass_at_range(const S5_PROFILE *profile, unsigned limit_mm)
{
    S5_BYPASS plan = {.failure = S5_FAIL_TARGET};
    if (!s5_range_measured(profile->nearest_mm) || profile->nearest_mm > limit_mm)
        return plan;
    /* Verify departure room before checking the sides. A fresh scan after a
       retrace can evaluate side clearance from farther back. */
    if (profile->front_mm < S5_TURN_CLEARANCE_MM) {
        plan.failure = S5_FAIL_ROOM;
        return plan;
    }

    unsigned left_offset = bypass_offset(-profile->left_edge_mm, profile->left_edge_valid);
    unsigned right_offset = bypass_offset(profile->right_edge_mm, profile->right_edge_valid);
    unsigned left_required = required_side_range(left_offset);
    unsigned right_required = required_side_range(right_offset);
    unsigned left_path = side_range(profile->left_path_mm, profile->left_mm, left_required);
    unsigned right_path = side_range(profile->right_path_mm, profile->right_mm, right_required);
    bool left_room = !profile->left_clipped && left_offset <= MAX_LATERAL_OFFSET_MM;
    bool right_room = !profile->right_clipped && right_offset <= MAX_LATERAL_OFFSET_MM;
    bool left = left_room && ((s5_range_available(left_path) && left_path >= left_required) ||
                              outward_route_fits(profile->left_path_mm, profile->left_outward_mm, left_offset));
    bool right = right_room && ((s5_range_available(right_path) && right_path >= right_required) ||
                                outward_route_fits(profile->right_path_mm, profile->right_outward_mm, right_offset));
    if (!left && !right) {
        bool left_unobserved = !outward_observed(profile->left_outward_mm);
        bool right_unobserved = !outward_observed(profile->right_outward_mm);
        plan.probe_left = left_room && left_unobserved && outward_probe_allowed(profile->left_path_mm, left_offset);
        plan.probe_right = right_room && right_unobserved && outward_probe_allowed(profile->right_path_mm, right_offset);
        if ((left_room && !s5_range_available(left_path) && left_unobserved) ||
            (right_room && !s5_range_available(right_path) && right_unobserved)) plan.failure = S5_FAIL_ECHO;
        else if (profile->left_clipped && profile->right_clipped) plan.failure = S5_FAIL_BOTH_LIMITS;
        else if (profile->left_clipped) plan.failure = S5_FAIL_LEFT_LIMIT;
        else if (profile->right_clipped) plan.failure = S5_FAIL_RIGHT_LIMIT;
        else plan.failure = left_room || right_room ? S5_FAIL_SIDE : S5_FAIL_ROOM;
        return plan;
    }
    /* Use right when both routes fit unless the measured left route is shorter. */
    bool shorter_left = left && profile->left_edge_valid && profile->right_edge_valid &&
                        left_offset + EDGE_OFFSET_DEADBAND_MM < right_offset;
    bool use_left = !right || shorter_left;
    plan.side = use_left ? S5_LEFT : S5_RIGHT;
    plan.failure = S5_FAIL_NONE;
    plan.offset_mm = use_left ? left_offset : right_offset;
    return plan;
}

S5_BYPASS s5_plan_bypass(const S5_PROFILE *profile)
{
    return s5_plan_bypass_at_range(profile, S5_FOLLOW_CLEAR_MM);
}

static S5_AVOID_OUTPUT advance(unsigned distance_mm)
{
    return (S5_AVOID_OUTPUT){.action = S5_ADVANCE, .distance_mm = distance_mm};
}

static void observe_line(S5_AVOID *state, const S5_AVOID_INPUT *in, bool seen, uint32_t since)
{
    if (at_or_after(in->line_at, since) && in->line_at != state->last_line_at) {
        state->last_line_at = in->line_at;
        state->line_samples = seen ? state->line_samples + 1u : 0u;
    }
}

static void record_tape_edge(S5_TAPE_EDGE *edge, bool dark, unsigned progress_mm)
{
    if (edge->samples == 2) return;
    if (!dark) {
        edge->samples = 0;
        return;
    }
    if (!edge->samples) edge->entry_mm = progress_mm;
    ++edge->samples;
}

static bool tape_edges_confirmed(const S5_AVOID *state)
{
    return state->left_edge.samples == 2 && state->right_edge.samples == 2;
}

static S5_AVOID_OUTPUT find_tape(S5_AVOID *state, const S5_AVOID_INPUT *in, bool settled)
{
    if (in->search_progress_mm > state->search_progress_mm)
        state->search_progress_mm = in->search_progress_mm;
    if (in->motion == S5_MOTION_DONE) state->search_exhausted = true;
    if ((in->near_original_line || state->search_exhausted) &&
        at_or_after(in->line_at, state->phase_at) && in->line_at != state->last_line_at) {
        state->last_line_at = in->line_at;
        record_tape_edge(&state->left_edge, in->left_on_tape, state->search_progress_mm);
        record_tape_edge(&state->right_edge, in->right_on_tape, state->search_progress_mm);
    }

    if (!state->align_at_mm && tape_edges_confirmed(state)) {
        unsigned left = state->left_edge.entry_mm;
        unsigned right = state->right_edge.entry_mm;
        unsigned separation = left > right ? left - right : right - left;
        /* Both sensors can reach the leading edge together on a perpendicular approach. */
        if (separation > S5_OUTER_IR_SPACING_MM)
            return fail(state, S5_FAIL_LINE, in->now);
        /* Bring the axle over the tape before the final pivot. */
        unsigned turn_start_mm = (left + right + 1u) / 2u + TAPE_HALF_WIDTH_MM + S5_IR_TO_AXLE_MM;
        if (turn_start_mm > S5_MAX_LEG_MM)
            return fail(state, S5_FAIL_LINE, in->now);
        state->align_at_mm = turn_start_mm;
    }

    /* Use the rest of the straight search move to reach the turn point.
       If the search ends first, CENTER_LINE supplies the remaining travel. */
    if (state->align_at_mm) {
        if (!state->search_exhausted && state->search_progress_mm < state->align_at_mm)
            return (S5_AVOID_OUTPUT){.action = S5_WAIT};
        go(state, S5_CENTER_LINE, in->now);
        return (S5_AVOID_OUTPUT){.action = S5_STOP};
    }

    if (state->search_exhausted && state->left_edge.samples != 1 && state->right_edge.samples != 1)
        return fail(state, S5_FAIL_LINE, in->now);
    if (!state->search_exhausted && settled && in->motion == S5_MOTION_NONE)
        return advance(state->search_mm);
    return (S5_AVOID_OUTPUT){.action = S5_WAIT};
}

static S5_AVOID_OUTPUT center_for_turn(S5_AVOID *state, const S5_AVOID_INPUT *in, bool settled)
{
    if (in->motion == S5_MOTION_DONE) return go(state, S5_ALIGN_LINE, in->now);
    if (!settled || in->motion != S5_MOTION_NONE) return (S5_AVOID_OUTPUT){.action = S5_WAIT};

    unsigned progress = in->search_progress_mm > state->search_progress_mm ?
                        in->search_progress_mm : state->search_progress_mm;
    int adjustment = (int)state->align_at_mm - (int)progress;
    if (adjustment >= -LINE_ALIGN_TOLERANCE_MM && adjustment <= LINE_ALIGN_TOLERANCE_MM)
        return go(state, S5_ALIGN_LINE, in->now);
    unsigned distance = (unsigned)(adjustment < 0 ? -adjustment : adjustment);
    unsigned limit = adjustment < 0 ? S5_SEARCH_MARGIN_MM : S5_IR_TO_AXLE_MM + TAPE_HALF_WIDTH_MM;
    if (distance > limit) return fail(state, S5_FAIL_LINE, in->now);
    return (S5_AVOID_OUTPUT){.action = adjustment < 0 ? S5_REVERSE : S5_ADVANCE,
                             .distance_mm = distance};
}

static S5_AVOID_OUTPUT execute_route(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    S5_PHASE phase = state->phase;
    uint32_t elapsed = in->now - state->phase_at;
    bool settled = in->stopped && elapsed >= MOTION_SETTLE_MS;
    unsigned travel = 0;
    if (phase == S5_SHIFT_OUT) travel = state->offset_mm;
    else if (phase == S5_PASS) travel = state->pass_mm;
    else if (phase == S5_FIND_LINE) travel = state->search_mm;
    /* Allow nominal travel at line-follow speed plus time to settle and accelerate. */
    unsigned timeout = PHASE_TIMEOUT_MS + travel * 60000u / (S3_CRUISE_RPM * WHEEL_CIRCUMFERENCE_MM);
    S5_AVOID_OUTPUT wait = {.action = S5_WAIT};
    if (elapsed >= timeout)
        return fail(state, phase == S5_CHECK_LINE || phase == S5_FIND_LINE || phase == S5_CENTER_LINE ?
                    S5_FAIL_LINE : S5_FAIL_PHASE_TIMEOUT, in->now);

    if (phase == S5_CHECK_LINE) {
        if (!settled) return wait;
        if (in->line_junction) return fail(state, S5_FAIL_LINE, in->now);
        /* After a measured crossing, the line follower can correct a tape
           edge that is still under one outer sensor after the final pivot. */
        bool at_tracking_position = in->line_centered || (in->left_on_tape != in->right_on_tape);
        observe_line(state, in, tape_edges_confirmed(state) && at_tracking_position,
                     state->phase_at + MOTION_SETTLE_MS);
        return state->line_samples >= 2 ? go(state, S5_COMPLETE, in->now) : wait;
    }

    if (phase == S5_FIND_LINE) return find_tape(state, in, settled);
    if (phase == S5_CENTER_LINE) return center_for_turn(state, in, settled);

    if (in->motion == S5_MOTION_DONE) {
        switch (phase) {
        case S5_TURN_OUT: return go(state, S5_SHIFT_OUT, in->now);
        case S5_SHIFT_OUT: return go(state, S5_ALIGN_OUTSIDE, in->now);
        case S5_ALIGN_OUTSIDE: return go(state, S5_PASS, in->now);
        case S5_PASS: return go(state, S5_TURN_IN, in->now);
        case S5_TURN_IN: return go(state, S5_FIND_LINE, in->now);
        case S5_ALIGN_LINE: return go(state, S5_CHECK_LINE, in->now);
        default: return fail(state, S5_FAIL_MOTION, in->now);
        }
    }

    if (!settled || in->motion != S5_MOTION_NONE) return wait;
    switch (phase) {
    case S5_SHIFT_OUT: return advance(state->offset_mm);
    case S5_PASS: return state->pass_mm ? advance(state->pass_mm) : go(state, S5_TURN_IN, in->now);
    case S5_TURN_OUT: case S5_ALIGN_OUTSIDE: case S5_TURN_IN: case S5_ALIGN_LINE: {
        bool outward = phase == S5_TURN_OUT || phase == S5_ALIGN_LINE;
        bool left = outward ? state->side == S5_LEFT : state->side == S5_RIGHT;
        return (S5_AVOID_OUTPUT){.action = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT};
    }
    default: return fail(state, S5_FAIL_MOTION, in->now);
    }
}

static S5_AVOID_OUTPUT wait_for_path(S5_AVOID *state, S5_FAILURE reason, const S5_AVOID_INPUT *in)
{
    state->failure = reason;
    state->failed_phase = state->phase;
    state->blocked_range_mm = S5_UNKNOWN_RANGE;
    if (in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS &&
        in->range_mm <= s5_scan_range_limit(state))
        state->blocked_range_mm = in->range_mm;
    return go(state, S5_WAIT_PATH, in->now);
}

static void clear_path_failure(S5_AVOID *state)
{
    state->failure = S5_FAIL_NONE;
    state->failed_phase = S5_OFF;
}

S5_AVOID_OUTPUT s5_avoid_step(S5_AVOID *state, const S5_AVOID_INPUT *in)
{
    S5_PHASE phase = state->phase;
    S5_AVOID_OUTPUT wait = {.action = S5_WAIT};
    if (phase == S5_OFF || phase == S5_COMPLETE || phase == S5_FAILED) return wait;
    if (in->motion == S5_MOTION_FAILED) return fail(state, S5_FAIL_MOTION, in->now);
    if (s5_avoid_executing(phase)) return execute_route(state, in);

    uint32_t elapsed = in->now - state->phase_at;
    unsigned clear_mm = s5_scan_range_limit(state);
    bool range_fresh = in->range_valid && in->now - in->range_at <= S5_RANGE_MAX_AGE_MS;
    if (phase == S5_WAIT_PATH) {
        /* The scan task keeps ranging at 90 degrees while stopped. A new
           sweep can begin once a current forward range justifies it. */
        if (!in->stopped) return wait;
        if (!range_fresh || !at_or_after(in->range_at, state->phase_at)) {
            state->range_seen = false;
            return wait;
        }
        if (in->range_mm > clear_mm) {
            if (!confirmed_forward(state, in)) return wait;
            clear_path_failure(state);
            return go(state, S5_COMPLETE, in->now);
        }
        if (in->retry_scan) {
            clear_path_failure(state);
            state->scan_retried = true;
            return go(state, S5_PROFILE_PATH, in->now);
        }
        state->range_seen = false;
        /* Allow one automatic retry if the forward echo returns or the
           measured clearance improves. A clear path can always resume above. */
        unsigned blocked = state->blocked_range_mm;
        if (state->scan_retried || state->failure == S5_FAIL_BOTH_LIMITS ||
            (s5_range_measured(blocked) && in->range_mm < blocked + RESCAN_RANGE_CHANGE_MM)) return wait;
        if (elapsed >= RESCAN_INTERVAL_MS) {
            clear_path_failure(state);
            state->scan_retried = true;
            return go(state, S5_PROFILE_PATH, in->now);
        }
        return wait;
    }
    unsigned timeout = phase == S5_PROFILE_PATH ? SCAN_TIMEOUT_MS : PHASE_TIMEOUT_MS;
    if (elapsed >= timeout) {
        S5_FAILURE reason = phase == S5_PROFILE_PATH && !in->profile_valid ? S5_FAIL_SCAN_TIMEOUT :
                            !range_fresh ? S5_FAIL_ECHO : S5_FAIL_PHASE_TIMEOUT;
        if (phase == S5_PROFILE_PATH || (phase == S5_BACK_UP && in->motion == S5_MOTION_NONE))
            return wait_for_path(state, reason, in);
        return fail(state, reason, in->now);
    }
    if (phase == S5_PROFILE_PATH) {
        /* The scan belongs to this stop and has completed with the servo centred. */
        if (!in->profile_valid || in->profile_at == state->phase_at ||
            !at_or_after(in->profile_at, state->phase_at) || !in->stopped) return wait;
        if (range_fresh && in->range_mm > clear_mm) {
            if (s5_range_measured(in->profile.nearest_mm) && !confirmed_forward(state, in)) return wait;
            clear_path_failure(state);
            return go(state, S5_COMPLETE, in->now);
        }
        state->range_seen = false;
        S5_PROFILE profile = in->profile;
        if (range_fresh && in->range_mm < profile.front_mm) profile.front_mm = in->range_mm;
        S5_BYPASS plan = s5_plan_bypass_at_range(&profile, clear_mm);
        if (plan.side != S5_NO_SIDE) {
            unsigned pass = s5_planned_pass_mm(&profile, range_fresh ? in->range_mm : S5_UNKNOWN_RANGE);
            if (pass > S5_MAX_PASS_MM) return wait_for_path(state, S5_FAIL_ROOM, in);
            unsigned offset = plan.offset_mm;
            clear_path_failure(state);
            state->side = plan.side;
            state->offset_mm = offset;
            /* The sideways leg and pivots make no forward progress along the tape. */
            state->pass_mm = pass;
            state->search_mm = offset + S5_SEARCH_MARGIN_MM + S5_SEARCH_EXTENSION_MM;
            state->search_after_mm = reentry_detection_mm(offset);
            return go(state, S5_TURN_OUT, in->now);
        }
        S5_FAILURE reason = plan.failure;
        if (reason == S5_FAIL_TARGET && !range_fresh)
            return wait_for_path(state, S5_FAIL_ECHO, in);
        if (!state->scan_retried && (reason == S5_FAIL_TARGET || reason == S5_FAIL_ECHO)) {
            state->scan_retried = true;
            go(state, S5_PROFILE_PATH, in->now);
            return (S5_AVOID_OUTPUT){.action = S5_RESCAN};
        }
        bool clipped = reason >= S5_FAIL_LEFT_LIMIT;
        bool both_clipped = reason == S5_FAIL_BOTH_LIMITS;
        /* Retrace only a verified straight segment. A single clipped edge gets
           a retry only if the target should stay within the normal window. */
        bool retrace_in_range = range_fresh && in->range_mm <= clear_mm - S5_REVERSE_MM;
        bool on_retrace = in->line_centered && !in->line_junction;
        if (state->reverse_available && range_fresh && on_retrace &&
            (both_clipped || !clipped || retrace_in_range)) {
            state->reverse_available = false;
            if (both_clipped) {
                /* A box can remain in the path beyond the original scan limit after retreating. */
                state->scan_after_retrace = state->scan_retried = true;
            } else {
                state->reverse_long = state->reverse_long && reason == S5_FAIL_ROOM &&
                                      profile.front_mm <= clear_mm - S5_LONG_REVERSE_MM &&
                                      in->range_mm <= clear_mm - S5_LONG_REVERSE_MM;
            }
            return go(state, S5_BACK_UP, in->now);
        }
        return wait_for_path(state, reason, in);
    }

    if (phase == S5_BACK_UP) {
        if (in->line_junction || !in->line_centered)
            return fail(state, S5_FAIL_LINE, in->now);
        if (!range_fresh)
            return in->motion == S5_MOTION_NONE ? wait_for_path(state, S5_FAIL_ECHO, in) :
                                                  fail(state, S5_FAIL_ECHO, in->now);
        unsigned reverse_mm = state->reverse_long ? S5_LONG_REVERSE_MM : S5_REVERSE_MM;
        if (in->motion == S5_MOTION_DONE) return go(state, S5_PROFILE_PATH, in->now);
        if (in->motion != S5_MOTION_NONE || !in->stopped || elapsed < MOTION_SETTLE_MS ||
            !at_or_after(in->range_at, state->phase_at + MOTION_SETTLE_MS)) return wait;
        return (S5_AVOID_OUTPUT){.action = S5_REVERSE,
                                 .distance_mm = reverse_mm};
    }

    return wait;
}
