#include "../../app_program/subsystem5/s5_echo.h"
#include "../../app_program/subsystem5/s5_profile.h"
#include <assert.h>
#include <stdio.h>

static void test_echo_pairing(void)
{
    _Static_assert(sizeof(S5_PING) == 4, "The echo result fits in four bytes");
    S5_ECHO_CAPTURE capture = {.triggered_at = UINT32_MAX - 50};
    s5_capture_edge(&capture, UINT32_MAX - 20, true, false, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_IDLE);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 20, true, false, S5_ECHO_TIMEOUT_US);
    s5_capture_edge(&capture, 895, false, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_DONE && capture.pulse_us == 875);
    S5_PING measured = s5_capture_result(&capture, 895, true, false);
    assert(measured.state == S5_ECHO_PULSE && measured.range_mm == 150);
    s5_capture_edge(&capture, 901, true, false, S5_ECHO_TIMEOUT_US);
    assert(capture.pulse_us == 875);

    capture = (S5_ECHO_CAPTURE){.triggered_at = 100, .state = S5_CAPTURE_RISE};
    s5_capture_edge(&capture, 110, false, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_BAD);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 110, true, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_BAD);
    capture.state = S5_CAPTURE_RISE;
    s5_capture_edge(&capture, 120, true, false, S5_ECHO_TIMEOUT_US);
    s5_capture_edge(&capture, 30100, false, true, S5_ECHO_TIMEOUT_US);
    assert(capture.state == S5_CAPTURE_FALL); /* trailing edge arrived too late */
    S5_PING fall = s5_capture_result(&capture, 30100, true, false);
    assert(fall.state == S5_ECHO_FALL_TIMEOUT && fall.range_mm == S5_UNKNOWN_RANGE);
    s5_capture_edge(&capture, 38120, false, true, 60000);
    S5_PING late = s5_capture_result(&capture, 38120, true, false);
    assert(late.state == S5_ECHO_LATE && late.pulse_us == 38000);
    assert(late.range_mm == S5_UNKNOWN_RANGE);
    capture.state = S5_CAPTURE_RISE;
    S5_PING rise = s5_capture_result(&capture, 30100, true, false);
    assert(rise.state == S5_ECHO_QUIET && rise.range_mm == S5_UNKNOWN_RANGE);
    assert(s5_capture_result(&capture, 30099, true, false).state == S5_ECHO_TIMING);
    capture.state = S5_CAPTURE_IDLE;
    S5_PING high = s5_capture_result(&capture, 100, true, true);
    assert(high.state == S5_ECHO_HIGH && high.range_mm == S5_UNKNOWN_RANGE);
    assert(s5_capture_result(&capture, 100, false, true).state == S5_ECHO_TIMING);
    S5_RANGE_GATE gate = {.range_mm = measured.range_mm};
    assert(s5_accept_range(&gate, fall.range_mm) && gate.blocked);
    assert(s5_classify_echo(S5_ECHO_PULSE, 25000).range_mm == S5_OPEN_RANGE);
}

static void test_rcwl_no_target(void)
{
    S5_ECHO_CAPTURE capture = {.triggered_at = 100, .state = S5_CAPTURE_RISE};
    s5_capture_edge(&capture, 400, true, false, S5_ECHO_OBSERVE_US);
    s5_capture_edge(&capture, 47495, false, true, S5_ECHO_OBSERVE_US);
    S5_PING ping = s5_capture_result(&capture, 47495, true, false);
    assert(ping.state == S5_ECHO_NO_TARGET && ping.pulse_us == 47095);
    assert(ping.range_mm == S5_OPEN_RANGE);
    assert(s5_range_from_echo(ping.pulse_us) == S5_UNKNOWN_RANGE);
    assert(s5_classify_echo(S5_ECHO_LATE, 46899).range_mm == S5_UNKNOWN_RANGE);
    assert(s5_classify_echo(S5_ECHO_LATE, 47301).range_mm == S5_UNKNOWN_RANGE);

    S5_RANGE_GATE gate = {.range_mm = 160, .blocked = true};
    assert(!s5_accept_range(&gate, ping.range_mm) && gate.blocked);
    assert(s5_accept_range(&gate, S5_UNKNOWN_RANGE) && gate.blocked);
    assert(!s5_accept_range(&gate, ping.range_mm) && gate.blocked);
    assert(s5_accept_range(&gate, ping.range_mm) && !gate.blocked);
}

static void test_scan_and_profile(void)
{
    _Static_assert(sizeof(S5_SCAN) == 38, "Pack the 25 12-bit samples into 38 bytes");
    assert(s5_servo_angle(0) == 150);
    assert(s5_servo_angle(S5_CENTRE_INDEX * S5_FINE_STEP) == 90);
    assert(s5_scan_bearing(S5_CENTRE_INDEX + 9) == 45);  /* R, right */
    assert(s5_scan_bearing(S5_CENTRE_INDEX - 9) == 135); /* T, left */
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) {
        s5_scan_set(&scan, i, 4000 + i);
    }
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) assert(s5_scan_get(&scan, i) == 4000 + i);
    assert(s5_scan_target(&scan, S5_CLEAR_MM) == S5_NO_TARGET);
    s5_scan_set(&scan, S5_CENTRE_INDEX, S5_FOLLOW_STOP_MM);
    assert(s5_scan_target(&scan, S5_FOLLOW_CLEAR_MM) == S5_CENTRE_INDEX);
    s5_scan_set(&scan, S5_CENTRE_INDEX, 392);
    assert(s5_scan_target(&scan, S5_FOLLOW_CLEAR_MM) == S5_NO_TARGET);
    s5_scan_set(&scan, S5_CENTRE_INDEX, S5_UNKNOWN_RANGE);
    s5_scan_set(&scan, 24, 140);
    assert(s5_scan_target(&scan, S5_CLEAR_MM) == 24);
    S5_PROFILE clipped = s5_profile(&scan, 23, 24, 24, 0);
    assert(s5_servo_angle(clipped.angle_offset_deg) == 30);
    assert(clipped.width_mm == S5_UNKNOWN_WIDTH && !clipped.right_edge_valid);
    s5_scan_set(&scan, 12, 150);
    assert(s5_scan_target(&scan, S5_CLEAR_MM) == 12);
    s5_scan_set(&scan, 10, 410);
    s5_scan_set(&scan, 11, 140);
    s5_scan_set(&scan, 13, 150);
    s5_scan_set(&scan, 14, S5_OPEN_RANGE);
    S5_PROFILE profile = s5_profile(&scan, 10, 14, 12, 0);
    assert(profile.nearest_mm == 140 && s5_servo_angle(profile.angle_offset_deg) == 95);
    assert(profile.width_mm == 14 && profile.right_mm == S5_OPEN_RANGE);
    assert(profile.left_edge_valid && profile.right_edge_valid);
    s5_scan_set(&scan, 14, S5_UNKNOWN_RANGE);
    profile = s5_profile(&scan, 10, 14, 12, 0);
    assert(profile.width_mm == 13 && !profile.right_edge_valid);
    assert(profile.width_status == S5_WIDTH_RIGHT_EDGE && profile.right_edge_mm == 13);
    s5_scan_set(&scan, 15, 550);
    profile = s5_profile(&scan, 10, 15, 12, 0);
    assert(profile.width_mm == 14 && profile.right_mm == 550);
    assert(profile.right_edge_valid && profile.right_edge_mm == 8);
    s5_scan_set(&scan, 15, 30); /* A much closer return cannot bracket the far edge. */
    s5_scan_set(&scan, 16, 550); /* Too far from the last hit to confirm the edge. */
    profile = s5_profile(&scan, 10, 16, 12, 0);
    assert(profile.width_mm == 13 && !profile.right_edge_valid);
}

static void test_observed_span_without_clearance(void)
{
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) s5_scan_set(&scan, i, S5_UNKNOWN_RANGE);
    for (unsigned i = 8; i <= 15; ++i) s5_scan_set(&scan, i, 176);
    s5_scan_set(&scan, 8, 189); /* 110 degrees, the last hit on the left */
    unsigned span = 0;
    S5_PROFILE profile = s5_profile(&scan, 5, 18, S5_CENTRE_INDEX, &span);
    assert(span == 109 && profile.nearest_mm == 176);
    assert(profile.width_mm == (span + 1u) / 2u && profile.width_status == S5_WIDTH_BOTH_EDGES);
    assert(!profile.left_edge_valid && !profile.right_edge_valid);
    assert(profile.left_edge_mm == -64 && profile.right_edge_mm == 45);
    profile = s5_profile(&scan, S5_CENTRE_INDEX, S5_CENTRE_INDEX, S5_CENTRE_INDEX, &span);
    assert(span == 0 && profile.width_mm == S5_UNKNOWN_WIDTH);
}

static void test_repeat_and_narrow_target(void)
{
    assert(s5_confirm_scan_range(160, 170) == 160);
    assert(s5_confirm_scan_range(160, 210) == S5_UNKNOWN_RANGE);
    assert(s5_confirm_scan_range(160, S5_OPEN_RANGE) == S5_UNKNOWN_RANGE);
    assert(s5_confirm_scan_range(S5_UNKNOWN_RANGE, S5_OPEN_RANGE) == S5_UNKNOWN_RANGE);
    assert(s5_confirm_scan_range(S5_OPEN_RANGE, S5_OPEN_RANGE) == S5_OPEN_RANGE);

    S5_SCAN scan = {0};
    s5_scan_set(&scan, 11, S5_OPEN_RANGE);
    s5_scan_set(&scan, 12, 300);
    s5_scan_set(&scan, 13, S5_OPEN_RANGE);
    unsigned span;
    S5_PROFILE profile = s5_profile(&scan, 11, 13, 12, &span);
    assert(span == 0 && profile.width_mm == S5_UNKNOWN_WIDTH && profile.front_mm == 300);
    assert(profile.width_status == S5_WIDTH_UNRESOLVED);
    assert(profile.left_edge_mm == -27 && profile.right_edge_mm == 27);
}

static void test_box_width_from_ray_brackets(void)
{
    S5_SCAN scan = {0};
    /* A 220 mm face at 209 mm has edges between the 25 and 30 degree rays. */
    const unsigned face_ranges[] = {209, 210, 212, 216, 222, 231, S5_OPEN_RANGE};
    for (unsigned i = 0; i < sizeof(face_ranges) / sizeof(*face_ranges); ++i) {
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, face_ranges[i]);
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, face_ranges[i]);
    }
    S5_PROFILE profile = s5_profile(&scan, 6, 18, S5_CENTRE_INDEX, NULL);
    /* The two measured 231 mm rays are 50 degrees apart. */
    assert(profile.width_mm == 98);
}

static void test_width_from_unequal_off_centre_edges(void)
{
    S5_SCAN scan = {0};
    s5_scan_set(&scan, 4, 810);
    s5_scan_set(&scan, 5, 180);
    s5_scan_set(&scan, 6, 185);
    s5_scan_set(&scan, 7, 190);
    s5_scan_set(&scan, 8, 205);
    s5_scan_set(&scan, 9, 220);
    s5_scan_set(&scan, 10, 650);

    S5_PROFILE profile = s5_profile(&scan, 4, 10, 7, NULL);
    /* Edges at 125 and 105 degrees give 79.85 mm at 180 and 220 mm. */
    assert(profile.width_mm == 40 && profile.width_status == S5_WIDTH_OK);
    assert(profile.left_mm == 810 && profile.right_mm == 650);
    assert(profile.left_edge_mm == -99 && profile.right_edge_mm == -59);
}

static void test_measured_width_with_wide_beam(void)
{
    S5_SCAN scan = {0};
    /* Wide bearings can overstate the physical width of a 220 mm face. */
    const unsigned left[] = {209, 209, 209, 209, 210, 213, 219, 224, 227, 229, 229, S5_OPEN_RANGE};
    const unsigned right[] = {209, 209, 209, 209, 210, 214, 221, 231, 240, 245, 245, 245, S5_OPEN_RANGE};
    for (unsigned i = 0; i < sizeof(left) / sizeof(*left); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, left[i]);
    for (unsigned i = 0; i < sizeof(right) / sizeof(*right); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, right[i]);
    S5_PROFILE profile = s5_profile(&scan, 1, 24, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 188 && profile.nearest_mm == 209);
    assert(profile.left_edge_mm == -81 && profile.right_edge_mm == 107);

    s5_scan_set(&scan, 3, S5_UNKNOWN_RANGE); /* Missing neighbour on the left. */
    s5_scan_set(&scan, 4, 229);              /* Another bearing still sees the corner. */
    s5_scan_set(&scan, 23, 211);             /* Rightmost echo came from the front. */
    profile = s5_profile(&scan, 1, 24, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 175 && profile.width_status == S5_WIDTH_OK);

    s5_scan_set(&scan, S5_CENTRE_INDEX, S5_UNKNOWN_RANGE);
    profile = s5_profile(&scan, 1, 24, S5_CENTRE_INDEX - 1, NULL);
    assert(profile.width_mm == 175);

    /* Equal edge ranges still give a chord when the bearings differ. */
    for (unsigned i = 2; i < 24; ++i) s5_scan_set(&scan, i, 209);
    profile = s5_profile(&scan, 1, 24, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 166 && profile.width_status == S5_WIDTH_OK);
}

static void test_narrow_face_at_track_distance(void)
{
    const unsigned left[] = {215, 215, 215, 216, 216, 217, 219, 220, 220, S5_OPEN_RANGE};
    const unsigned right[] = {215, 215, 215, 216, 216, 218, 219, 219, S5_OPEN_RANGE};
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < sizeof(left) / sizeof(*left); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, left[i]);
    for (unsigned i = 0; i < sizeof(right) / sizeof(*right); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, right[i]);
    S5_PROFILE profile = s5_profile(&scan, 3, 20, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 134 && profile.width_status == S5_WIDTH_OK);

    s5_scan_set(&scan, 20, 219);
    profile = s5_profile(&scan, 3, 20, S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 141 && profile.width_status == S5_WIDTH_RIGHT_EDGE);
}

static void test_track_left_limit(void)
{
    const unsigned sweep[][25] = {
        {240, 249, 240, 227, 228, 234, 230, 228, 229, 229, 228, 227, 220,
         218, 221, 220, 222, 221, 218, 222, 230, 231, 234, 235, 925},
        {245, 249, 240, 232, 225, 228, 230, 230, 222, 225, 228, 228, 219,
         218, 221, 221, 221, 226, 229, 229, 230, 230, 240, 232, 942},
    };
    for (unsigned pass = 0; pass < sizeof(sweep) / sizeof(*sweep); ++pass) {
        S5_SCAN scan = {0};
        for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) s5_scan_set(&scan, i, sweep[pass][i]);
        assert(s5_scan_target(&scan, S5_CLEAR_MM) == S5_CENTRE_INDEX);
        S5_PROFILE profile = s5_profile(&scan, 0, S5_SCAN_POINTS - 1, S5_CENTRE_INDEX, NULL);
        assert(profile.width_mm == (pass == 0 ? 200 : 201) && profile.width_status == S5_WIDTH_LEFT_EDGE);
        assert(profile.left_clipped && !profile.right_clipped);
        assert(!profile.left_edge_valid && profile.right_edge_valid);
        assert(profile.nearest_mm == 218 && profile.right_mm == sweep[pass][24]);
        assert(profile.front_mm >= 190 && profile.right_edge_mm < 110);
    }
}

static void test_bench_background_returns(void)
{
    const unsigned mixed_at_135[] = {486, 2000, 231, 2009, 829};
    const unsigned distant_at_140[] = {2005, 821, 2005, 800, 2013};
    const unsigned repeated_at_145[] = {1996, S5_UNKNOWN_RANGE, 1997, S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE};
    const unsigned scattered_open[] = {S5_OPEN_RANGE, S5_UNKNOWN_RANGE, S5_OPEN_RANGE, S5_UNKNOWN_RANGE, S5_OPEN_RANGE};
    const unsigned confirmed_open[] = {S5_UNKNOWN_RANGE, S5_OPEN_RANGE, S5_OPEN_RANGE, S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE};
    assert(s5_edge_background(mixed_at_135, 5, 197) == S5_UNKNOWN_RANGE);
    assert(s5_edge_background(distant_at_140, 5, 197) == 800);
    assert(s5_edge_background(repeated_at_145, 5, 197) == 1996);
    assert(s5_edge_background(scattered_open, 5, 197) == S5_UNKNOWN_RANGE);
    assert(s5_edge_background(confirmed_open, 5, 197) == S5_OPEN_RANGE);

    /* Even agreeing background echoes cannot hide a later near return. */
    const unsigned late_near[] = {800, 801, 820, 845, 231};
    assert(s5_edge_background(late_near, 5, 197) == S5_UNKNOWN_RANGE);

    const unsigned bench_sweep[] = {218, 209, 204, 200, 200, 200, 199, 199, 197,
                                    195, 195, 195, 195, 199, 204, 199, 203, 208, 917};
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < sizeof(bench_sweep) / sizeof(*bench_sweep); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX - 8 + i, bench_sweep[i]);
    s5_scan_set(&scan, S5_CENTRE_INDEX - 9, s5_edge_background(mixed_at_135, 5, 197));
    s5_scan_set(&scan, S5_CENTRE_INDEX - 10, s5_edge_background(distant_at_140, 5, 197));
    S5_PROFILE profile = s5_profile(&scan, S5_CENTRE_INDEX - 10, S5_CENTRE_INDEX + 10,
                                    S5_CENTRE_INDEX, NULL);
    assert(profile.left_edge_valid && profile.right_edge_valid);
    assert(profile.width_mm == 144 && profile.width_status == S5_WIDTH_OK);
}

static void test_80_mm_bench_target_with_unstable_corner_ranges(void)
{
    const unsigned left[] = {190, 194, 194, 194, 198, 194, 199, 199, 203, 212,
                             S5_UNKNOWN_RANGE, 630};
    const unsigned right[] = {190, 189, 190, 194, 194, 194, 194, 198, 207, 1196};
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < sizeof(left) / sizeof(*left); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, left[i]);
    for (unsigned i = 0; i < sizeof(right) / sizeof(*right); ++i)
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, right[i]);
    S5_PROFILE profile = s5_profile(&scan, S5_CENTRE_INDEX - 11, S5_CENTRE_INDEX + 9,
                                    S5_CENTRE_INDEX, NULL);
    assert(profile.nearest_mm == 189 && profile.left_edge_valid && profile.right_edge_valid);
    assert(profile.left_mm == 630 && profile.right_mm == 1196);
    assert(profile.left_edge_mm == -79 && profile.right_edge_mm == 63);
    /* The broad beam reported edge hits 85 degrees apart on the 80 mm target. */
    assert(profile.width_mm == 142 && profile.width_status == S5_WIDTH_OK);
}

static void test_centered_box_edge_coordinates(void)
{
    S5_SCAN scan = {0};
    for (unsigned i = 0; i <= 12; ++i) {
        unsigned range = i < 11 ? 230 : i == 11 ? 244 : S5_OPEN_RANGE;
        s5_scan_set(&scan, S5_CENTRE_INDEX - i, range);
        s5_scan_set(&scan, S5_CENTRE_INDEX + i, range);
    }
    S5_PROFILE profile = s5_profile(&scan, S5_CENTRE_INDEX - 12, S5_CENTRE_INDEX + 12,
                                    S5_CENTRE_INDEX, NULL);
    assert(profile.width_mm == 200 && profile.left_edge_valid && profile.right_edge_valid);
    assert(profile.left_edge_mm == -100 && profile.right_edge_mm == 100);
    assert(profile.front_mm > 200); /* Wide returns are outside the forward corridor. */
}

static void test_track_replay_sees_no_edge_on_either_side(void)
{
    const unsigned distances[] = {279, 259, 252, 251, 241, 233, 232, 233, 233, 227, 222, 221, 222,
                                  220, 215, 223, 223, 223, 227, 221, 230, 220, 224, 234, 243};
    S5_SCAN scan = {0};
    for (unsigned i = 0; i < sizeof(distances) / sizeof(*distances); ++i)
        s5_scan_set(&scan, i, distances[i]);
    S5_PROFILE profile = s5_profile(&scan, 0, S5_SCAN_POINTS - 1u, S5_CENTRE_INDEX, NULL);
    assert(profile.nearest_mm == 215 && profile.width_mm == 226);
    assert(profile.left_clipped && profile.right_clipped &&
           !profile.left_edge_valid && !profile.right_edge_valid);
}

static void test_clearance_with_unsteady_background(void)
{
    const unsigned varied[] = {910, S5_UNKNOWN_RANGE, 705, 1160, 835};
    assert(s5_clearance_range(varied, 5) == 705);
    const unsigned incomplete[] = {S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE, 800, S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE};
    assert(s5_clearance_range(incomplete, 5) == S5_UNKNOWN_RANGE);
    const unsigned no_returns[] = {S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE, S5_UNKNOWN_RANGE};
    assert(s5_clearance_range(no_returns, 3) == S5_UNKNOWN_RANGE);
    const unsigned open[] = {S5_OPEN_RANGE, S5_UNKNOWN_RANGE, S5_OPEN_RANGE};
    assert(s5_clearance_range(open, 3) == S5_OPEN_RANGE);
    assert(s5_clearance_range(open, 2) == S5_UNKNOWN_RANGE);
    const unsigned mixed[] = {S5_OPEN_RANGE, S5_UNKNOWN_RANGE, 705};
    assert(s5_clearance_range(mixed, 3) == 705);
    const unsigned near[] = {95, S5_OPEN_RANGE, S5_OPEN_RANGE};
    assert(s5_clearance_range(near, 3) == 95);
}

int main(void)
{
    test_centered_box_edge_coordinates();
    test_track_replay_sees_no_edge_on_either_side();
    test_echo_pairing();
    test_rcwl_no_target();
    test_scan_and_profile();
    test_observed_span_without_clearance();
    test_repeat_and_narrow_target();
    test_box_width_from_ray_brackets();
    test_width_from_unequal_off_centre_edges();
    test_measured_width_with_wide_beam();
    test_narrow_face_at_track_distance();
    test_track_left_limit();
    test_bench_background_returns();
    test_clearance_with_unsteady_background();
    test_80_mm_bench_target_with_unstable_corner_ranges();
    puts("PASS echo capture, scan packing, partial width estimates, sweep limits and bench background replay");
}
