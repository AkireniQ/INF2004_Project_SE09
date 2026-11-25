#include <assert.h>
#include <stdio.h>

#include "../../app_program/subsystem3/barcode.h"
#include "../../app_program/subsystem3/line_steering.h"
#include "../../app_program/vehicle_controller/vehicle_core.h"

static void check_steering(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (int sample = 0; sample < 40; ++sample) {
        int noise = sample % 2 == 0 ? 60 : -60;
        line_decide(250 + noise, sample % 2 == 0 ? 1000 : 0, 250 - noise, &state, &out);
        assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM && !out.stop);
    }
    line_decide(40, 420, 980, &state, &out);
    line_decide(250, 420, 250, &state, &out);
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);
    assert(state.curve_side == 0);
}

static void check_steering_reversal(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (unsigned i = 0; i < 12u; ++i) {
        line_decide(130, 0, 475, &state, &out);
    }
    assert(out.left > out.right);
    unsigned straight = 0;
    for (unsigned i = 0; i < 30u; ++i) {
        line_decide(380, 0, 130, &state, &out);
        if (out.left == out.right) {
            ++straight;
        } else if (out.left < out.right) {
            assert(straight >= 8u);
            return;
        }
    }
    assert(0 && "persistent error must still steer after the neutral interval");
}

static void check_early_release(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (unsigned i = 0; i < 20u; ++i) {
        line_decide(130, 0, 425, &state, &out);
    }
    assert(state.curve_side == 0 && out.left - out.right == S3_CORRECTION_STEP_RPM);
    for (unsigned i = 0; i < 3u; ++i) {
        line_decide(130, 0, 225, &state, &out);
    }
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);
}

static void check_brief_edge_touches(bool reading_barcode)
{
    LF_STATE state = {.reading_barcode = reading_barcode};
    LF_CMD out;
    unsigned length = reading_barcode ? 20u : 12u;
    for (unsigned i = 0; i < 6u; ++i) {
        int left = i % 2u ? 920 : 120;
        int right = i % 2u ? 120 : 920;
        for (unsigned sample = 0; sample < length; ++sample) {
            line_decide(left, 0, right, &state, &out);
            assert(!out.stop && !state.curve_side);
            assert(out.left >= S3_CRUISE_RPM - 2 && out.left <= S3_CRUISE_RPM);
            assert(out.right >= S3_CRUISE_RPM - 2 && out.right <= S3_CRUISE_RPM);
        }
    }
    for (unsigned i = 0; i < 45u; ++i) line_decide(120, 0, 920, &state, &out);
    assert(state.curve_side == 1 && !out.stop);
    if (reading_barcode) {
        assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM - S3_CORRECTION_STEP_RPM);
        state.reading_barcode = false;
        for (unsigned i = 0; i < 15u; ++i) line_decide(120, 0, 920, &state, &out);
    }
    assert(out.right == 0);
}

static void check_barcode_at_crawl(unsigned narrow_um)
{
    /* F at the speed produced by the steering wheel targets. */
    const char *pattern = "nwnnwnwnn" "n" "nnwnwwnnn" "n" "nwnnwnwnn";
    LF_STATE state = {0};
    LF_CMD out;
    unsigned width = 22000u, position = 0, element = 0, edge_age = 0;
    bool steered_left = false, steered_right = false;
    int edge_side = 1;
    char letter = 0;
    barcode_reset();
    for (uint32_t now = 0; now < 12000u && !letter; now += 10u) {
        while (position >= width && element < BARCODE_FRAME_ELEMENTS + 1u) {
            ++element;
            if (element <= BARCODE_FRAME_ELEMENTS)
                width += pattern[element - 1u] == 'w' ? 3u * narrow_um : narrow_um;
        }
        bool dark = element % 2u != 0 && element <= BARCODE_FRAME_ELEMENTS;
        letter = barcode_feed(dark ? 3000 : 220, now);
        bool reading = barcode_progress().elements >= 3u;
        if (reading != state.reading_barcode || edge_age >= (reading ? 20u : 12u)) {
            edge_age = 0;
            edge_side = -edge_side;
        }
        state.reading_barcode = reading;
        ++edge_age;
        line_decide(edge_side < 0 ? 920 : 120, 0, edge_side > 0 ? 920 : 120, &state, &out);
        assert(!out.stop && !state.curve_side);
        assert(out.left >= S3_CRUISE_RPM - 2 && out.right >= S3_CRUISE_RPM - 2);
        assert(out.left - out.right <= 2 && out.right - out.left <= 2);
        if (reading) {
            steered_left |= out.left < out.right;
            steered_right |= out.left > out.right;
        }
        position += (unsigned)(out.left + out.right) * 210u * 1000u / 12000u;
    }
    assert(letter == 'F');
    assert(steered_left && steered_right);
}

static void check_curve(int side)
{
    LF_STATE state = {0};
    LF_CMD out;
    int left = side < 0 ? 950 : 80;
    int right = side > 0 ? 950 : 80;

    for (int sample = 0; sample < 35; ++sample) {
        int before = state.correction_rpm;
        line_decide(left, 600, right, &state, &out);
        assert((state.correction_rpm - before) * side >= 0);
        assert((state.correction_rpm - before) * side <= S3_CORRECTION_STEP_RPM);
        assert(out.left >= 0 && out.left <= S3_MAX_OUTER_RPM);
        assert(out.right >= 0 && out.right <= S3_MAX_OUTER_RPM);
        assert(!(out.left & 1) && !(out.right & 1) && !out.stop);
    }
    assert(state.curve_side == side);
    assert((side < 0 ? out.left : out.right) == 0);

    for (unsigned sample = 0; sample < 30u; ++sample) {
        line_decide(30, 950, 30, &state, &out);
        assert(!out.stop && state.curve_side == side);
        assert(side * (out.left - out.right) > 0);
    }

    /* A recovered line returns to neutral without flipping the correction. */
    for (int sample = 0; sample < 20; ++sample) {
        line_decide(430, 950, 430, &state, &out);
        assert(!out.stop && side * (out.left - out.right) >= 0);
    }
    assert(state.curve_side == 0);
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);
}

static void check_line_drop(void)
{
    LF_STATE state = {0};
    LF_CMD out;

    for (unsigned sample = 0; sample < S3_CURVE_CONFIRM_SAMPLES + 2u; ++sample) {
        line_decide(80, 600, 950, &state, &out);
    }
    assert(state.curve_side == 1);
    for (unsigned sample = 0; sample < 30u; ++sample) {
        line_decide(30, 1000, 30, &state, &out);
        assert(!out.stop);
    }
    assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);

    for (unsigned sample = 0; sample < 35u; ++sample) {
        line_decide(950, 0, 80, &state, &out);
    }
    for (unsigned sample = 0; sample < S3_BLIND_CURVE_LIMIT - 1u; ++sample) {
        line_decide(20, 0, 20, &state, &out);
        assert(!out.stop);
    }
    for (unsigned sample = 0; sample < 120u; ++sample) {
        line_decide(20, 0, 20, &state, &out);
        assert(out.stop && out.left == 0 && out.right == 0);
    }

    state = (LF_STATE){0};
    for (int sample = 0; sample < 30; ++sample) {
        line_decide(810, 0, 1000, &state, &out);
    }
    assert(out.left == out.right && !out.stop);
}

static void check_curve_exit(int side, bool next_bend)
{
    LF_STATE state = {0};
    LF_CMD out;
    const int left = side < 0 ? 950 : 80;
    const int right = side > 0 ? 950 : 80;

    for (unsigned i = 0; i < 35u; ++i) {
        line_decide(left, 0, right, &state, &out);
    }
    for (unsigned i = 0; i < 2u; ++i) {
        line_decide(430, 0, 430, &state, &out);
    }
    assert(state.curve_side == 0 && state.settle_samples == S3_CURVE_EXIT_SAMPLES);

    /* A brief opposite edge after turning gets only a small heading trim. */
    for (unsigned i = 0; i < 18u; ++i) {
        line_decide(right, 0, left, &state, &out);
        assert(state.curve_side == 0 && !out.stop);
        assert(side * (out.left - out.right) >= -S3_CORRECTION_STEP_RPM);
    }

    if (next_bend) {
        for (unsigned i = 0; i < 12u; ++i) {
            line_decide(right, 0, left, &state, &out);
        }
        assert(state.curve_side == -side && !out.stop);
    } else {
        for (unsigned i = 0; i < 40u; ++i) {
            line_decide(250, 0, 250, &state, &out);
        }
        assert(state.curve_side == 0 && !out.stop);
        assert(out.left == S3_CRUISE_RPM && out.right == S3_CRUISE_RPM);
    }
}

static void check_start_stop(void)
{
    VC_MODEL model = {0};
    VC_OUTPUT out;
    VC_INPUT input = {
        .start = true, .ready = true, .line_valid = true,
        .left_rpm = 30, .right_rpm = 30
    };
    vc_core_step(&model, &input, &out);
    assert(!vc_core_running(&model) && out.stop);
    input.link_ok = true;
    vc_core_step(&model, &input, &out);
    assert(vc_core_running(&model) && !out.stop && out.left_rpm == 30 && out.right_rpm == 30);
    input.start = false;
    input.link_ok = false;
    vc_core_step(&model, &input, &out);
    assert(vc_core_running(&model) && !out.stop);
    input.line_age_ms = VC_LINE_MAX_AGE_MS + 1u;
    vc_core_step(&model, &input, &out);
    assert(!vc_core_running(&model) && out.stop && model.reason == VC_REASON_STALE_LINE);
    input.line_age_ms = 0;
    input.link_ok = true;
    vc_core_step(&model, &input, &out);
    assert(out.stop);
    input.start = true;
    input.stop = true;
    vc_core_step(&model, &input, &out);
    assert(!vc_core_running(&model) && out.stop && model.reason == VC_REASON_STOP);
    input.start = false;
    input.stop = false;
    vc_core_step(&model, &input, &out);
    assert(out.stop);
    input.start = true;
    input.motor_faults = 1;
    vc_core_step(&model, &input, &out);
    assert(out.stop && model.reason == VC_REASON_MOTOR);
}

int main(void)
{
    check_steering();
    check_steering_reversal();
    check_early_release();
    check_brief_edge_touches(false);
    check_brief_edge_touches(true);
    check_barcode_at_crawl(6990u);
    check_barcode_at_crawl(5160u);
    check_barcode_at_crawl(3509u);
    check_curve(-1);
    check_curve(1);
    check_line_drop();
    check_curve_exit(-1, false);
    check_curve_exit(1, false);
    check_curve_exit(-1, true);
    check_curve_exit(1, true);
    check_start_stop();
    puts("PASS straight trim, curve entry and exit, recovery timeout, MQTT start and STOP priority");
    return 0;
}
