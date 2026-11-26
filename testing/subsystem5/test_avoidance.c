/* Host tests for the production controller and planner. */
#include "../../app_program/vehicle_controller/vehicle_core.h"
#include "../../app_program/subsystem2/motion_config.h"
#include <assert.h>
#include <stdio.h>

static void step(VC_MODEL *model, VC_INPUT *input, VC_OUTPUT *output, unsigned ms)
{
    input->avoidance.now += ms;
    if (input->avoidance.range_valid) input->avoidance.range_at = input->avoidance.now;
    vc_core_step(model, input, output);
}

static VC_INPUT track_input(void)
{
    return (VC_INPUT){
        .ready = true, .line_valid = true, .link_ok = true,
        .left_rpm = 30, .right_rpm = 30,
        .avoidance = {.stopped = true, .range_valid = true, .range_mm = 1000},
    };
}

static void profile(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, S5_PROFILE scan)
{
    in->start = true;
    step(model, in, out, 20);
    assert(model->state == VC_FOLLOW_LINE && !out->stop);
    in->start = false;
    in->avoidance.range_mm = S5_FOLLOW_STOP_MM + 1;
    step(model, in, out, 20);
    assert(out->left_rpm == 20 && out->right_rpm == 20);

    S5_RANGE_GATE gate = {0};
    assert(s5_accept_range(&gate, S5_SCAN_STOP_MM + 1) && !gate.blocked);
    assert(s5_accept_range(&gate, S5_SCAN_STOP_MM) && gate.blocked);
    assert(s5_accept_range(&gate, S5_CLEAR_MM) && gate.blocked);
    assert(s5_accept_range(&gate, S5_CLEAR_MM + 1) && !gate.blocked);
    in->obstacle_blocked = true;
    in->avoidance.range_mm = scan.nearest_mm > S5_FOLLOW_STOP_MM ? scan.nearest_mm : S5_FOLLOW_STOP_MM;
    step(model, in, out, 20);
    assert(out->stop && model->state == VC_AVOIDING && model->avoidance.phase == S5_PROFILE_PATH);

    in->avoidance.profile = scan;
    in->avoidance.profile_valid = true;
    in->avoidance.profile_at = in->avoidance.now - 20;
    step(model, in, out, 20);
    assert(model->avoidance.phase == S5_PROFILE_PATH); /* ignore pre-stop scan */
    in->avoidance.profile_at = in->avoidance.now + 2000;
    step(model, in, out, 2000);
}

static S5_PROFILE obstacle(bool left)
{
    return (S5_PROFILE){.nearest_mm = S5_FOLLOW_STOP_MM, .front_mm = S5_FOLLOW_STOP_MM, .width_mm = 90,
                        .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE,
                        .left_path_mm = left ? 850 : 600,
                        .right_path_mm = left ? S5_MANEUVER_STOP_MM : 850,
                        .left_edge_mm = -50, .right_edge_mm = 40,
                        .left_edge_valid = true, .right_edge_valid = true};
}

static void start_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                       S5_ACTION action, unsigned travel)
{
    in->avoidance.motion = S5_MOTION_NONE;
    step(model, in, out, 260);
    if (out->maneuver.action != action || out->maneuver.distance_mm != travel)
        fprintf(stderr, "phase=%u range=%u motion=%u/%u expected=%u/%u\n",
                model->avoidance.phase, in->avoidance.range_mm, out->maneuver.action,
                out->maneuver.distance_mm, action, travel);
    assert(out->maneuver.action == action && out->maneuver.distance_mm == travel);
    in->avoidance.motion = S5_MOTION_PENDING;
}

static void end_move(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out)
{
    in->avoidance.motion = S5_MOTION_DONE;
    step(model, in, out, 500);
    in->avoidance.motion = S5_MOTION_NONE;
}

static void reach_inward_turn(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out, bool left, bool open_side)
{
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    S5_PROFILE scan = obstacle(left);
    if (open_side) {
        scan.width_mm = S5_UNKNOWN_WIDTH;
        if (left) {
            scan.left_path_mm = S5_OPEN_RANGE;
            scan.right_clipped = true;
            scan.right_edge_valid = false;
        } else {
            scan.right_path_mm = S5_OPEN_RANGE;
            scan.left_clipped = true;
            scan.left_edge_valid = false;
        }
    }
    profile(model, in, out, scan);
    assert(model->avoidance.side == (left ? S5_LEFT : S5_RIGHT));
    in->obstacle_blocked = false; /* Dispatcher clears the old heading's obstacle latch. */
    in->avoidance.range_valid = false;
    in->avoidance.range_mm = S5_UNKNOWN_RANGE;

    start_move(model, in, out, outward, 0);
    end_move(model, in, out);
    unsigned offset = left ? 185u : 175u;
    assert(model->avoidance.offset_mm == offset);
    start_move(model, in, out, S5_ADVANCE, offset);
    end_move(model, in, out);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    unsigned pass = s5_planned_pass_mm(&scan, S5_FOLLOW_STOP_MM);
    start_move(model, in, out, S5_ADVANCE, pass);
    end_move(model, in, out);
    assert(model->avoidance.phase == S5_TURN_IN && model->avoidance.pass_mm == pass);
    start_move(model, in, out, inward, 0);
    end_move(model, in, out);
    assert(model->state == VC_RECOVER_LINE && model->avoidance.phase == S5_FIND_LINE);
    assert(model->avoidance.search_after_mm == (left ? 6u : 0u));
}

static void tape_sample(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                        unsigned progress, bool near, bool far)
{
    bool left_bypass = model->avoidance.side == S5_LEFT;
    in->avoidance.left_on_tape = left_bypass ? far : near;
    in->avoidance.right_on_tape = left_bypass ? near : far;
    in->avoidance.search_progress_mm = progress;
    in->avoidance.line_at = in->avoidance.now + 20;
    step(model, in, out, 20);
}

static void cross_tape_and_center(VC_MODEL *model, VC_INPUT *in, VC_OUTPUT *out,
                                  unsigned near_entry, unsigned far_entry, unsigned turn_start)
{
    start_move(model, in, out, S5_ADVANCE, model->avoidance.search_mm);

    tape_sample(model, in, out, 0, true, false);
    assert(model->avoidance.left_edge.samples == 0 && model->avoidance.right_edge.samples == 0);
    in->avoidance.near_original_line = true;
    tape_sample(model, in, out, near_entry, true, false);
    step(model, in, out, 20); /* Reused timestamp must not confirm the hit. */
    assert(model->avoidance.left_edge.samples + model->avoidance.right_edge.samples == 1);
    tape_sample(model, in, out, near_entry + 1u, true, false);
    assert(model->avoidance.phase == S5_FIND_LINE && !out->stop);

    tape_sample(model, in, out, far_entry, true, true);
    assert(model->avoidance.phase == S5_FIND_LINE && !out->stop);
    tape_sample(model, in, out, far_entry + 1u, true, true);
    assert(model->avoidance.align_at_mm == turn_start);
    assert(model->avoidance.phase == S5_FIND_LINE && out->maneuver.action == S5_WAIT && !out->stop);
    tape_sample(model, in, out, turn_start - 1u, false, false);
    assert(model->avoidance.phase == S5_FIND_LINE && !out->stop);
    tape_sample(model, in, out, turn_start, false, false);
    assert(model->avoidance.phase == S5_CENTER_LINE && out->maneuver.action == S5_STOP);

    in->avoidance.motion = S5_MOTION_NONE;
    in->avoidance.search_progress_mm = turn_start + 1u;
    step(model, in, out, 260);
    assert(model->avoidance.phase == S5_ALIGN_LINE);
}

static void test_rejoin(bool left, bool open_side)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    reach_inward_turn(&model, &in, &out, left, open_side);
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    /* Allow slight skew, then wait until the axle reaches the tape before turning. */
    cross_tape_and_center(&model, &in, &out, 28, 44, 175);
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_CHECK_LINE);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 260);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    step(&model, &in, &out, 20);
    assert(model.state == VC_RECOVER_LINE); /* crossing evidence cannot authorize handoff */
    in.avoidance.line_centered = true;
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
    step(&model, &in, &out, 20);
    assert(!out.stop && out.left_rpm == 30 && out.right_rpm == 30);
    in.avoidance.range_valid = true;
    in.avoidance.range_mm = 1000;
    in.obstacle_blocked = false;
    step(&model, &in, &out, 20);
    assert(!out.stop && out.left_rpm == 30 && out.right_rpm == 30);
    in.avoidance.range_mm = 180;
    in.obstacle_blocked = true;
    step(&model, &in, &out, 20);
    assert(out.stop && model.avoidance.phase == S5_PROFILE_PATH);
}

static void test_abort_paths(void)
{
    enum { OPERATOR_STOP, STALE_LINE, MOTOR_REJECTED, NO_LINE };
    for (unsigned cause = OPERATOR_STOP; cause <= NO_LINE; ++cause) {
        VC_MODEL model = {0};
        VC_INPUT in = track_input();
        VC_OUTPUT out;
        reach_inward_turn(&model, &in, &out, false, false);
        unsigned travel = model.avoidance.search_mm;
        start_move(&model, &in, &out, S5_ADVANCE, travel);
        switch (cause) {
        case OPERATOR_STOP: in.stop = true; break;
        case STALE_LINE: in.line_age_ms = VC_LINE_MAX_AGE_MS + 1; break;
        case MOTOR_REJECTED: in.avoidance.motion = S5_MOTION_FAILED; break;
        case NO_LINE: in.avoidance.motion = S5_MOTION_DONE; break;
        }
        step(&model, &in, &out, 20);
        assert(out.stop && !vc_core_running(&model) && model.avoidance.phase == S5_FAILED);
        in = track_input();
        in.start = true;
        step(&model, &in, &out, 20);
        assert(out.stop && !vc_core_running(&model)); /* repositioning needs actual IR evidence */
    }
}

static void test_ir_gap_during_encoder_route(S5_SIDE side)
{
    VC_MODEL model = {.state = VC_AVOIDING, .avoidance = {
        .phase = S5_ALIGN_OUTSIDE, .side = side, .pass_mm = 360, .search_mm = 180,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_ACTION inward = side == S5_LEFT ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    in.avoidance.range_valid = false;
    start_move(&model, &in, &out, inward, 0);

    in.line_age_ms = 170;
    step(&model, &in, &out, 170);
    assert(!out.stop && model.avoidance.phase == S5_ALIGN_OUTSIDE);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PASS);
    in.line_valid = false;
    start_move(&model, &in, &out, S5_ADVANCE, 360);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, inward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_FIND_LINE);

    step(&model, &in, &out, 260);
    assert(out.stop && out.maneuver.action == S5_WAIT && model.state == VC_RECOVER_LINE);
    VC_MODEL persistent_gap = model;
    VC_INPUT still_stale = in;
    step(&persistent_gap, &still_stale, &out, VC_LINE_REACQUIRE_WAIT_MS - 260);
    assert(out.stop && persistent_gap.state == VC_FAULT &&
           persistent_gap.reason == VC_REASON_STALE_LINE);

    in.line_valid = true;
    in.line_age_ms = 0;
    in.avoidance.line_at = in.avoidance.now + 10;
    step(&model, &in, &out, 10);
    assert(!out.stop && out.maneuver.action == S5_ADVANCE && out.maneuver.distance_mm == 180);
}

static void test_link_loss_does_not_end_a_run(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.start = true;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE);
    in.start = false;
    in.link_ok = false;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && !out.stop && out.left_rpm == 30);

    VC_MODEL lost_ir = model;
    VC_INPUT missing_sample = in;
    missing_sample.line_age_ms = VC_LINE_MAX_AGE_MS + 1;
    step(&lost_ir, &missing_sample, &out, 20);
    assert(out.stop && lost_ir.state == VC_FAULT && lost_ir.reason == VC_REASON_STALE_LINE);

    in.obstacle_blocked = true;
    in.avoidance.range_mm = 230;
    step(&model, &in, &out, 20);
    assert(model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);
    in.avoidance.profile = obstacle(true);
    in.avoidance.profile_valid = true;
    in.avoidance.profile_at = in.avoidance.now + 600;
    step(&model, &in, &out, 600);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
    in.stop = true;
    step(&model, &in, &out, 20);
    assert(out.stop && model.state == VC_IDLE);
}

static void test_unsafe_profile_and_scan_timeout(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(false);
    scan.left_path_mm = 150;
    scan.right_path_mm = 190;
    profile(&model, &in, &out, scan);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH &&
           model.avoidance.failure == S5_FAIL_SIDE && model.state == VC_AVOIDING);
    step(&model, &in, &out, 1000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
    in.avoidance.range_mm += 30;
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_PROFILE_PATH);

    model = (VC_MODEL){0};
    in = track_input();
    in.start = true;
    step(&model, &in, &out, 20);
    in.start = false;
    in.obstacle_blocked = true;
    step(&model, &in, &out, 20);
    VC_MODEL lost_target = model;
    VC_INPUT unavailable = in;
    unavailable.avoidance.range_valid = false;
    unavailable.avoidance.profile_valid = true;
    unavailable.avoidance.profile_at = unavailable.avoidance.now + 20;
    unavailable.avoidance.profile.nearest_mm = S5_UNKNOWN_RANGE;
    step(&lost_target, &unavailable, &out, 20);
    assert(out.maneuver.action == S5_WAIT && lost_target.avoidance.phase == S5_WAIT_PATH &&
           lost_target.avoidance.failure == S5_FAIL_ECHO);

    step(&model, &in, &out, 15000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH &&
           model.avoidance.failure == S5_FAIL_SCAN_TIMEOUT && model.state == VC_AVOIDING);
    in.avoidance.range_valid = false;
    step(&model, &in, &out, 1000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
    in.avoidance.range_valid = true;
    in.avoidance.range_mm = 196;
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_PROFILE_PATH);
}

static void test_reentry_limit(void)
{
    S5_AVOID state = {.phase = S5_FIND_LINE, .side = S5_LEFT, .search_mm = 185};
    S5_AVOID_INPUT in = {.now = 300, .stopped = true};
    assert(s5_avoid_step(&state, &in).distance_mm == 185);
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_STOP);
    assert(state.phase == S5_FAILED && state.failure == S5_FAIL_LINE);

    /* If the first taped sample arrives at the encoder limit, wait for
       confirmation and never issue the same bounded search again. */
    state = (S5_AVOID){.phase = S5_FIND_LINE, .side = S5_LEFT, .search_mm = 185};
    in.line_at = in.now;
    in.right_on_tape = true;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.search_exhausted);
    in.motion = S5_MOTION_NONE;
    ++in.now;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    in.line_at = in.now;
    in.right_on_tape = false;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_LINE);
}

static void test_complete_profile_commits_without_another_echo(void)
{
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100};
    S5_AVOID_INPUT in = {.now = 10000, .profile_at = 10000, .profile_valid = true,
                        .profile = obstacle(true), .stopped = true, .range_mm = S5_UNKNOWN_RANGE};
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
    in.now += 260;
    assert(s5_avoid_step(&state, &in).action == S5_PIVOT_LEFT);
}

static void test_pivot_stays_bounded(void)
{
    S5_AVOID state = {.phase = S5_TURN_OUT, .side = S5_LEFT};
    S5_AVOID_INPUT in = {.motion = S5_MOTION_PENDING, .now = 9000};
    assert(s5_avoid_step(&state, &in).action == S5_STOP);
    assert(state.phase == S5_FAILED && state.failure == S5_FAIL_PHASE_TIMEOUT);

    state = (S5_AVOID){.phase = S5_PASS, .pass_mm = 510};
    in.now = 15000;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PASS);
    in.now = 24000;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_PHASE_TIMEOUT);
}

static void test_tape_beyond_original_search_limit(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE, .avoidance = {
        .phase = S5_FIND_LINE, .side = S5_RIGHT, .search_mm = 184 + S5_SEARCH_EXTENSION_MM,
    }};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.range_valid = false;
    in.avoidance.near_original_line = true;
    start_move(&model, &in, &out, S5_ADVANCE, 244);
    tape_sample(&model, &in, &out, 184, false, false);
    assert(model.avoidance.phase == S5_FIND_LINE && !out.stop);

    /* Verify the same stripe if the physical far sensor happens to hit first. */
    tape_sample(&model, &in, &out, 192, false, true);
    tape_sample(&model, &in, &out, 193, false, true);
    assert(model.avoidance.phase == S5_FIND_LINE && !out.stop);
    tape_sample(&model, &in, &out, 220, true, false);
    tape_sample(&model, &in, &out, 221, true, false);
    assert(!out.stop && model.avoidance.phase == S5_FIND_LINE && model.avoidance.align_at_mm == 345);

    /* The original straight command finishes before the measured turn point.
       Complete the remaining encoder distance before pivoting. */
    in.avoidance.search_progress_mm = 244;
    end_move(&model, &in, &out);
    assert(out.stop && model.avoidance.phase == S5_CENTER_LINE);
    start_move(&model, &in, &out, S5_ADVANCE, 101);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_ALIGN_LINE);
}

static void test_clipped_hold_ignores_interleaved_forward_outliers(void)
{
    S5_AVOID state = {.phase = S5_WAIT_PATH, .phase_at = 100, .failure = S5_FAIL_BOTH_LIMITS,
                      .blocked_range_mm = 220};
    S5_AVOID_INPUT in = {.now = 10000, .range_at = 10000, .stopped = true, .range_valid = true};
    const unsigned observations[] = {400, 229, 258, 218, 235, 476, 230, 420};
    for (unsigned i = 0; i < sizeof(observations) / sizeof(*observations); ++i) {
        in.range_mm = observations[i];
        in.range_at = ++in.now;
        assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_WAIT_PATH);
    }
    in.range_mm = 420;
    in.range_at = ++in.now;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_COMPLETE);
}

static void test_width_and_side_clearance(void)
{
    S5_PROFILE scan = obstacle(true);
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT);
    assert(plan.offset_mm == 185);
    scan.left_path_mm = scan.right_path_mm = S5_OPEN_RANGE;
    scan.left_edge_mm = -100;
    scan.right_edge_mm = 70;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT); /* Shorter shift leaves more space. */
    scan.left_edge_mm = -scan.right_edge_mm;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT); /* Comparable open routes prefer right. */
    scan.front_mm = S5_TURN_CLEARANCE_MM - 1u;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_ROOM);
    scan = obstacle(true);
    scan.front_mm = S5_SCAN_STOP_MM;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 185);
    scan = obstacle(false);
    scan.left_path_mm = S5_UNKNOWN_RANGE;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT);

    scan.front_mm = 215;
    scan.left_path_mm = 150;
    scan.right_path_mm = 222;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);
}

static void test_centered_and_offset_boxes(void)
{
    S5_PROFILE scan = obstacle(true);
    scan.width_mm = 200;
    scan.left_path_mm = scan.right_path_mm = 450;
    scan.left_edge_mm = -100;
    scan.right_edge_mm = 100;
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 235);

    scan.left_path_mm = scan.right_path_mm = 399;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);
    scan.right_path_mm = 400;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 235);

    scan.left_edge_mm = -150;
    scan.right_edge_mm = 70;
    scan.left_path_mm = 450;
    scan.right_path_mm = 400;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 70 + S5_SIDE_ALLOWANCE_MM);
    scan.left_edge_mm = -70;
    scan.right_edge_mm = 150;
    scan.left_path_mm = 400;
    scan.right_path_mm = S5_OPEN_RANGE; /* A distant return used to outweigh the shorter left route. */
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 70 + S5_SIDE_ALLOWANCE_MM);
    scan.left_path_mm = 100;
    scan.left_mm = S5_OPEN_RANGE;
    plan = s5_plan_bypass(&scan); /* A close return on the left rules it out even with distant background. */
    assert(plan.side == S5_RIGHT);

    scan.left_edge_mm = -220;
    scan.right_edge_mm = -20;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 147);
    scan.left_edge_mm = -250;
    scan.right_edge_mm = -50;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 147);
}

static void test_bypass_with_estimated_edges(void)
{
    S5_PROFILE scan = obstacle(true);
    scan.left_edge_valid = scan.right_edge_valid = false;
    scan.left_edge_mm = -60;
    scan.right_edge_mm = 50;
    scan.width_mm = 28;
    scan.left_path_mm = 650;
    scan.right_path_mm = 730;

    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.failure == S5_FAIL_NONE);
    unsigned right_offset = 50 + S5_SIDE_ALLOWANCE_MM + S5_UNCERTAIN_EDGE_MM;
    assert(plan.offset_mm == right_offset);
    scan.width_mm = 342;
    assert(s5_plan_bypass(&scan).offset_mm == right_offset); /* Never use the whole width for one side. */

    scan.left_path_mm = S5_OPEN_RANGE;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT && plan.offset_mm == 60 + S5_SIDE_ALLOWANCE_MM + S5_UNCERTAIN_EDGE_MM);

    scan.left_path_mm = 150;
    scan.right_path_mm = right_offset + 15 + S5_MANEUVER_STOP_MM - 1;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);

    scan.left_clipped = scan.right_clipped = true;
    scan.left_path_mm = scan.right_path_mm = S5_OPEN_RANGE;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_BOTH_LIMITS);
}

static void test_no_route_with_unconfirmed_clearance(void)
{
    S5_PROFILE scan = obstacle(true);
    scan.left_path_mm = S5_UNKNOWN_RANGE;
    scan.right_path_mm = 210;
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_ECHO);

    scan.left_mm = S5_OPEN_RANGE;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_LEFT);
    scan.left_path_mm = 140;
    assert(s5_plan_bypass(&scan).side == S5_NO_SIDE);
}

static void test_outward_clearance(void)
{
    S5_PROFILE scan = obstacle(false);
    scan.left_edge_mm = -100;
    scan.right_edge_mm = 100;
    scan.left_path_mm = 140;
    scan.right_path_mm = 360;
    scan.right_outward_mm = S5_UNKNOWN_RANGE;

    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_SIDE);
    assert(!plan.probe_left && plan.probe_right);

    scan.right_outward_mm = 381;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && !plan.probe_right);
    scan.right_outward_mm = 382; /* Minimum range for 330 mm of lateral room. */
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 235);
    assert(!plan.probe_left && !plan.probe_right);

    scan.front_mm = S5_TURN_CLEARANCE_MM - 1u;
    plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_NO_SIDE && plan.failure == S5_FAIL_ROOM);
    scan.front_mm = S5_SCAN_STOP_MM;
    scan.right_outward_mm = S5_OPEN_RANGE;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    assert(s5_plan_bypass(&scan).side == S5_RIGHT);
    scan.right_path_mm = 300; /* The 45-degree echo blocks the departure. */
    assert(s5_plan_bypass(&scan).side == S5_NO_SIDE);
    scan.right_clipped = true;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    assert(s5_plan_bypass(&scan).side == S5_NO_SIDE);

    scan.left_path_mm = 360;
    assert(s5_plan_bypass(&scan).probe_left);
    scan.left_outward_mm = 390;
    assert(s5_plan_bypass(&scan).side == S5_LEFT);
}

static void test_left_fallback_clearance(void)
{
    S5_PROFILE scan = obstacle(true);
    scan.left_path_mm = S5_UNKNOWN_RANGE;
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100, .reverse_available = true};
    S5_AVOID_INPUT in = {
        .now = 500, .range_at = 500, .profile_at = 500, .range_mm = S5_FOLLOW_STOP_MM,
        .range_valid = true, .profile_valid = true, .stopped = true, .profile = scan,
        .line_centered = true,
    };
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    assert(state.phase == S5_TURN_OUT && state.side == S5_LEFT && state.check_left);
    in.now = 760;
    assert(s5_avoid_step(&state, &in).action == S5_PIVOT_LEFT);
    in.now = 3600;
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    assert(state.phase == S5_SHIFT_OUT && !s5_avoid_ranging_paused(&state));

    in.motion = S5_MOTION_NONE;
    in.now = 3860;
    in.range_at = 3859;
    in.range_mm = 600;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT);
    S5_AVOID blocked = state;
    S5_AVOID no_echo = state;
    in.range_at = in.now;
    in.range_mm = state.offset_mm - 1;
    assert(s5_avoid_step(&blocked, &in).action == S5_STOP && blocked.failure == S5_FAIL_CLOSE);
    in.range_valid = false;
    assert(s5_avoid_step(&no_echo, &in).action == S5_WAIT);
    in.now = 5100;
    assert(s5_avoid_step(&no_echo, &in).action == S5_STOP && no_echo.failure == S5_FAIL_ECHO);

    in.range_valid = true;
    in.range_at = in.now = 3870;
    in.range_mm = 600;
    S5_AVOID_OUTPUT move = s5_avoid_step(&state, &in);
    assert(move.action == S5_ADVANCE && move.distance_mm == 185 && s5_avoid_ranging_paused(&state));
}

static void test_lateral_bypass_at_start(void)
{
    /* Reproduce the sweep with short 45-degree readings and usable space at R. */
    S5_PROFILE scan = {.nearest_mm = S5_FOLLOW_STOP_MM, .front_mm = S5_FOLLOW_STOP_MM, .width_mm = 250,
                       .left_mm = S5_UNKNOWN_RANGE, .right_mm = 925,
                       .left_path_mm = 234, .right_path_mm = 230, .left_clipped = true,
                       .right_edge_mm = 234, .right_edge_valid = true};
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.side == S5_RIGHT && plan.offset_mm == 234 + S5_SIDE_ALLOWANCE_MM);

    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.start = in.obstacle_blocked = true;
    in.avoidance.range_mm = S5_FOLLOW_STOP_MM;
    assert(vc_core_start_block(&model, &in) == VC_START_READY);
    step(&model, &in, &out, 20);
    assert(out.stop && model.state == VC_AVOIDING && model.avoidance.phase == S5_PROFILE_PATH);

    in.start = false;
    in.avoidance.profile = scan;
    in.avoidance.profile_valid = true;
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_SHIFT_OUT);

    VC_MODEL clear = model;
    VC_INPUT roomy = in;
    roomy.avoidance.range_mm = 1000;
    start_move(&clear, &roomy, &out, S5_ADVANCE, 369);
    end_move(&clear, &roomy, &out);
    start_move(&clear, &roomy, &out, S5_PIVOT_LEFT, 0);
    end_move(&clear, &roomy, &out);
    assert(clear.avoidance.pass_mm == 475);
    start_move(&clear, &roomy, &out, S5_ADVANCE, 475);
    end_move(&clear, &roomy, &out);
    assert(clear.avoidance.phase == S5_TURN_IN);

    in.avoidance.range_mm = 60;
    start_move(&model, &in, &out, S5_ADVANCE, 369);
}

static void test_stationary_retry(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(true);
    scan.left_path_mm = 80;
    scan.right_path_mm = S5_UNKNOWN_RANGE;
    scan.width_mm = S5_UNKNOWN_WIDTH;
    profile(&model, &in, &out, scan);
    assert(out.maneuver.action == S5_RESCAN && model.avoidance.phase == S5_PROFILE_PATH);
    step(&model, &in, &out, 20);
    assert(out.maneuver.action == S5_WAIT);
    in.avoidance.profile = obstacle(true);
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    assert(model.avoidance.phase == S5_TURN_OUT && model.avoidance.side == S5_LEFT);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
}

static void test_clipped_scan(bool can_retrace)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = true;
    in.avoidance.can_retrace = can_retrace;
    S5_PROFILE scan = obstacle(true);
    scan.width_mm = S5_UNKNOWN_WIDTH;
    scan.left_edge_valid = false;
    scan.left_clipped = true;
    scan.left_path_mm = 234;
    scan.right_path_mm = 230;
    profile(&model, &in, &out, scan);
    if (!can_retrace) {
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
        assert(model.avoidance.failure == S5_FAIL_LEFT_LIMIT);
        return;
    }
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_BACK_UP);
    start_move(&model, &in, &out, S5_REVERSE, S5_REVERSE_MM);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PROFILE_PATH);
    in.avoidance.range_mm += S5_REVERSE_MM;
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
    assert(model.avoidance.failure == S5_FAIL_LEFT_LIMIT);
}

static void test_departure_room_after_retrace(void)
{
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100,
                      .reverse_available = true, .reverse_long = true};
    S5_AVOID_INPUT in = {.now = 500, .profile_at = 450, .range_at = 500, .range_mm = 90,
                         .profile_valid = true, .stopped = true, .range_valid = true,
                         .line_centered = true, .profile = obstacle(true)};
    /* The newer forward measurement leaves too little room to pivot. */
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_BACK_UP);
    in.now = in.range_at = 800;
    S5_AVOID_OUTPUT out = s5_avoid_step(&state, &in);
    assert(out.action == S5_REVERSE && out.distance_mm == S5_LONG_REVERSE_MM);
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    in.now = in.range_at = in.profile_at = 950;
    in.motion = S5_MOTION_NONE;
    in.range_mm = 150;
    in.profile.front_mm = in.profile.nearest_mm = 150;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
}

static void test_clipped_retrace_keeps_box_in_range(void)
{
    S5_PROFILE scan = {.nearest_mm = 221, .front_mm = 206, .width_mm = 231,
                       .left_clipped = true, .right_clipped = true,
                       .left_path_mm = 241, .right_path_mm = 240,
                       .left_mm = S5_UNKNOWN_RANGE, .right_mm = S5_UNKNOWN_RANGE};
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100};
    S5_AVOID_INPUT in = {.now = 500, .profile_at = 450, .range_at = 500, .range_mm = 221,
                        .profile_valid = true, .stopped = true, .range_valid = true,
                        .line_centered = true, .profile = scan};
    S5_AVOID no_trail = state;
    assert(s5_avoid_step(&no_trail, &in).action == S5_WAIT && no_trail.phase == S5_WAIT_PATH);
    assert(no_trail.failure == S5_FAIL_BOTH_LIMITS);

    state.reverse_available = state.reverse_long = true;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_BACK_UP);
    in.now = in.range_at = 800;
    S5_AVOID_OUTPUT move = s5_avoid_step(&state, &in);
    assert(move.action == S5_REVERSE && move.distance_mm == 60);
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    assert(s5_scan_range_limit(&state) == 330);

    in.motion = S5_MOTION_NONE;
    in.now = in.profile_at = in.range_at = 1000;
    in.profile.nearest_mm = in.range_mm = 281;
    S5_AVOID second_clipped = state;
    assert(s5_avoid_step(&second_clipped, &in).action == S5_WAIT && second_clipped.phase == S5_WAIT_PATH);
    for (unsigned i = 0; i < 2; ++i) {
        in.now = in.range_at = in.now + 1000;
        assert(s5_avoid_step(&second_clipped, &in).action == S5_WAIT);
        assert(second_clipped.phase == S5_WAIT_PATH && second_clipped.failure == S5_FAIL_BOTH_LIMITS);
    }

    /* The same obstacle now has a verified right route at the farther position. */
    in.profile.right_clipped = false;
    in.profile.right_edge_valid = true;
    in.profile.right_edge_mm = 95;
    in.profile.right_path_mm = 1000;
    in.profile_at = in.now;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
    assert(state.side == S5_RIGHT && state.phase == S5_TURN_OUT);
}

static void test_close_left_obstacle_with_right_clearance(bool has_retrace)
{
    S5_AVOID state = {.phase = S5_PROFILE_PATH, .phase_at = 100,
                      .reverse_available = has_retrace, .reverse_long = has_retrace};
    S5_AVOID_INPUT in = {
        .now = 500, .profile_at = 490, .range_at = 500, .range_mm = 64,
        .profile_valid = true, .stopped = true, .range_valid = true, .line_centered = true,
        .profile = {.nearest_mm = 64, .front_mm = 58, .width_mm = 78, .left_clipped = true,
                    .left_path_mm = 68, .right_path_mm = 839, .right_mm = S5_UNKNOWN_RANGE},
    };
    S5_AVOID_OUTPUT out = s5_avoid_step(&state, &in);
    assert(out.action == S5_WAIT);
    if (!has_retrace) {
        assert(state.phase == S5_WAIT_PATH && state.failure == S5_FAIL_ROOM);
        in.now = in.range_at = 8000;
        assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_WAIT_PATH);
        in.now = in.range_at = 8100;
        in.range_mm = 124;
        assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
        return;
    }
    assert(state.phase == S5_BACK_UP);
    in.now = in.range_at = 800;
    out = s5_avoid_step(&state, &in);
    assert(out.action == S5_REVERSE && out.distance_mm == 60);

    in.now = in.range_at = 1500;
    in.motion = S5_MOTION_DONE;
    in.range_mm = 124;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_PROFILE_PATH);
    in.now = in.range_at = in.profile_at = 2500;
    in.motion = S5_MOTION_NONE;
    in.profile.nearest_mm = 124;
    in.profile.front_mm = 118;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_TURN_OUT);
    assert(state.side == S5_RIGHT);
}

static void test_reverse_and_rescan(bool path_clears)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = in.avoidance.can_retrace = true;
    S5_PROFILE blocked = obstacle(true);
    blocked.front_mm = blocked.nearest_mm = 90;
    profile(&model, &in, &out, blocked);
    assert(model.avoidance.phase == S5_BACK_UP);
    start_move(&model, &in, &out, S5_REVERSE, S5_REVERSE_MM);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_PROFILE_PATH);
    step(&model, &in, &out, 20);
    assert(model.avoidance.phase == S5_PROFILE_PATH); /* previous scan can't be reused */

    if (path_clears) {
        in.avoidance.profile = (S5_PROFILE){.nearest_mm = S5_OPEN_RANGE};
        in.avoidance.range_mm = S5_FOLLOW_CLEAR_MM + 1;
        in.obstacle_blocked = false;
    }
    in.avoidance.profile_at = in.avoidance.now + 2000;
    step(&model, &in, &out, 2000);
    if (!path_clears) {
        assert(out.maneuver.action == S5_WAIT && model.avoidance.phase == S5_WAIT_PATH);
        return;
    }
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
}

static void test_square_bypass(bool left)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(left);
    scan.width_mm = 180;
    scan.left_edge_mm = -90;
    scan.right_edge_mm = 90;
    if (!left) {
        scan.left_path_mm = 140;
        scan.right_path_mm = 360;
        scan.right_outward_mm = 390;
    }
    S5_BYPASS plan = s5_plan_bypass(&scan);
    assert(plan.offset_mm == 225);
    profile(&model, &in, &out, scan);
    assert(model.avoidance.phase == S5_TURN_OUT);
    S5_ACTION outward = left ? S5_PIVOT_LEFT : S5_PIVOT_RIGHT;
    S5_ACTION inward = left ? S5_PIVOT_RIGHT : S5_PIVOT_LEFT;
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    in.avoidance.range_mm = S5_OPEN_RANGE;
    in.obstacle_blocked = false;
    start_move(&model, &in, &out, S5_ADVANCE, 225);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, inward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.offset_mm == plan.offset_mm && model.avoidance.pass_mm == 440);
    start_move(&model, &in, &out, S5_ADVANCE, 440);
    assert(model.avoidance.phase == S5_PASS);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_TURN_IN);
    in.avoidance.range_valid = false;
    start_move(&model, &in, &out, inward, 0);
    end_move(&model, &in, &out);
    assert(model.avoidance.search_mm == 225 + S5_SEARCH_MARGIN_MM + S5_SEARCH_EXTENSION_MM);
    start_move(&model, &in, &out, S5_ADVANCE, model.avoidance.search_mm);
    in.avoidance.near_original_line = true;
    tape_sample(&model, &in, &out, 86, true, true);
    tape_sample(&model, &in, &out, 87, true, true);
    assert(model.avoidance.align_at_mm == 225 && model.avoidance.phase == S5_FIND_LINE);
    tape_sample(&model, &in, &out, 224, false, false);
    assert(model.avoidance.phase == S5_FIND_LINE && !out.stop);
    tape_sample(&model, &in, &out, 225, false, false);
    assert(model.avoidance.phase == S5_CENTER_LINE && out.maneuver.action == S5_STOP);
    in.avoidance.motion = S5_MOTION_NONE;
    step(&model, &in, &out, 260);
    assert(model.avoidance.phase == S5_ALIGN_LINE);
    start_move(&model, &in, &out, outward, 0);
    end_move(&model, &in, &out);
    in.avoidance.line_hit = true;
    in.avoidance.line_centered = false;
    in.avoidance.left_on_tape = !left;
    in.avoidance.right_on_tape = left;
    in.avoidance.line_at = in.avoidance.now + 260;
    step(&model, &in, &out, 260);
    step(&model, &in, &out, 20);
    assert(model.state == VC_RECOVER_LINE && model.avoidance.line_samples == 1);
    in.avoidance.line_at = in.avoidance.now + 20;
    step(&model, &in, &out, 20);
    assert(model.state == VC_FOLLOW_LINE && model.avoidance.phase == S5_COMPLETE);
    step(&model, &in, &out, 20);
    assert(!out.stop);
}

static void test_blank_floor_after_alignment_stops(void)
{
    VC_MODEL model = {.state = VC_RECOVER_LINE, .avoidance = {.phase = S5_CHECK_LINE}};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    in.avoidance.line_centered = true;
    in.avoidance.line_at = 300;
    step(&model, &in, &out, 300);
    assert(model.state == VC_RECOVER_LINE && out.maneuver.action == S5_WAIT);
    step(&model, &in, &out, 9000);
    assert(out.stop && model.state == VC_FAULT && model.avoidance.failure == S5_FAIL_LINE);
}

static void test_tape_already_under_sensor_at_start_of_search(void)
{
    S5_AVOID state = {.phase = S5_FIND_LINE, .side = S5_LEFT, .search_mm = 90};
    S5_AVOID_INPUT in = {.stopped = true, .near_original_line = true, .right_on_tape = true,
                         .now = 260, .line_at = 260};
    assert(s5_avoid_step(&state, &in).action == S5_ADVANCE && state.right_edge.samples == 1);
    in.now = in.line_at = 270;
    in.motion = S5_MOTION_PENDING;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.right_edge.samples == 2);
    assert(state.phase == S5_FIND_LINE); /* The other edge has yet to cross. */
}

static void test_alignment_offset_bounds(void)
{
    S5_AVOID state = {.phase = S5_CENTER_LINE, .align_at_mm = 124, .search_progress_mm = 118};
    S5_AVOID_INPUT in = {.stopped = true, .now = 260};
    S5_AVOID_OUTPUT out = s5_avoid_step(&state, &in);
    assert(out.action == S5_ADVANCE && out.distance_mm == 6);
    in.motion = S5_MOTION_DONE;
    assert(s5_avoid_step(&state, &in).action == S5_WAIT && state.phase == S5_ALIGN_LINE);

    state = (S5_AVOID){.phase = S5_CENTER_LINE, .align_at_mm = 124, .search_progress_mm = 180};
    in.motion = S5_MOTION_NONE;
    assert(s5_avoid_step(&state, &in).action == S5_STOP && state.failure == S5_FAIL_LINE);
}

static void test_plan_pass_distance(void)
{
    S5_PROFILE scan = obstacle(false);
    scan.nearest_mm = 220;
    scan.width_mm = 200;
    assert(s5_planned_pass_mm(&scan, 230) == 500);
    assert(s5_planned_pass_mm(&scan, 190) == 490);
    scan.width_status = S5_WIDTH_RIGHT_EDGE;
    assert(s5_planned_pass_mm(&scan, 230) == 525);
    scan.width_mm = 280;
    assert(s5_planned_pass_mm(&scan, 230) == 565);
    scan.width_mm = S5_UNKNOWN_WIDTH;
    assert(s5_planned_pass_mm(&scan, 230) == 520);

    scan.width_status = S5_WIDTH_OK;
    scan.width_mm = 670;
    assert(s5_planned_pass_mm(&scan, 220) == S5_MAX_PASS_MM);
    scan.width_mm = 682;
    assert(s5_planned_pass_mm(&scan, 220) == S5_MAX_PASS_MM + 1u);

    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    scan = obstacle(false);
    scan.width_mm = 850;
    profile(&model, &in, &out, scan);
    assert(model.avoidance.phase == S5_WAIT_PATH && model.avoidance.failure == S5_FAIL_ROOM);
    assert(out.maneuver.action == S5_WAIT);
}

static void test_compact_bypass(void)
{
    VC_MODEL model = {0};
    VC_INPUT in = track_input();
    VC_OUTPUT out;
    S5_PROFILE scan = obstacle(false);
    scan.left_edge_mm = -240;
    scan.right_edge_mm = -50;
    profile(&model, &in, &out, scan);
    assert(model.avoidance.side == S5_RIGHT && model.avoidance.offset_mm == 147);
    start_move(&model, &in, &out, S5_PIVOT_RIGHT, 0);
    end_move(&model, &in, &out);
    in.avoidance.range_mm = S5_OPEN_RANGE;
    start_move(&model, &in, &out, S5_ADVANCE, 147);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
    end_move(&model, &in, &out);
    start_move(&model, &in, &out, S5_ADVANCE, 395);
    end_move(&model, &in, &out);
    assert(model.avoidance.phase == S5_TURN_IN);
    start_move(&model, &in, &out, S5_PIVOT_LEFT, 0);
}

int main(void)
{
    test_rejoin(true, false);
    test_rejoin(false, false);
    test_rejoin(true, true);
    test_rejoin(false, true);
    test_abort_paths();
    test_ir_gap_during_encoder_route(S5_LEFT);
    test_ir_gap_during_encoder_route(S5_RIGHT);
    test_link_loss_does_not_end_a_run();
    test_unsafe_profile_and_scan_timeout();
    test_reentry_limit();
    test_complete_profile_commits_without_another_echo();
    test_pivot_stays_bounded();
    test_tape_beyond_original_search_limit();
    test_clipped_hold_ignores_interleaved_forward_outliers();
    test_width_and_side_clearance();
    test_centered_and_offset_boxes();
    test_bypass_with_estimated_edges();
    test_no_route_with_unconfirmed_clearance();
    test_outward_clearance();
    test_left_fallback_clearance();
    test_lateral_bypass_at_start();
    test_stationary_retry();
    test_clipped_scan(false);
    test_clipped_scan(true);
    test_clipped_retrace_keeps_box_in_range();
    test_departure_room_after_retrace();
    test_close_left_obstacle_with_right_clearance(true);
    test_close_left_obstacle_with_right_clearance(false);
    test_reverse_and_rescan(true);
    test_reverse_and_rescan(false);
    test_square_bypass(true);
    test_square_bypass(false);
    test_plan_pass_distance();
    test_compact_bypass();
    test_blank_floor_after_alignment_stops();
    test_tape_already_under_sensor_at_start_of_search();
    test_alignment_offset_bounds();
    puts("PASS pivot and tight-space bypasses, side clearance, tape confirmation and cancellation");
}
