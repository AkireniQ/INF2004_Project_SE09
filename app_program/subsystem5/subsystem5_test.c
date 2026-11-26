#include "subsystem5.h"
#include "s5_profile.h"
#include "../subsystem1/s1_wire.h"
#include "../test_report.h"

#if APP_TEST
EXPORT void subsystem5_test(void)
{
    TEST_INIT();
    S5_SCAN samples = {0};
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) s5_scan_set(&samples, i, S5_UNKNOWN_RANGE);
    const unsigned face_ranges[] = {200, 201, 203, 207};
    for (unsigned i = 0; i < sizeof(face_ranges) / sizeof(*face_ranges); ++i) {
        s5_scan_set(&samples, S5_CENTRE_INDEX - i, face_ranges[i]);
        s5_scan_set(&samples, S5_CENTRE_INDEX + i, face_ranges[i]);
    }
    s5_scan_set(&samples, S5_CENTRE_INDEX - 4, 410);
    s5_scan_set(&samples, S5_CENTRE_INDEX + 4, S5_OPEN_RANGE);
    S5_PROFILE profile = s5_profile(&samples, S5_CENTRE_INDEX - 5, S5_CENTRE_INDEX + 5, S5_CENTRE_INDEX, 0);
    TEST_CHECK("S5-U01", profile.nearest_mm == 200 && profile.angle_offset_deg == 60 &&
               profile.width_mm == 54 && profile.left_mm == 410 &&
               profile.right_mm == S5_OPEN_RANGE && profile.right_edge_valid,
               "profile distinguishes an open side from an electrical fault");
    profile = s5_profile(&samples, S5_CENTRE_INDEX, S5_CENTRE_INDEX + 5, S5_CENTRE_INDEX, 0);
    TEST_CHECK("S5-U02", profile.width_mm == 27 && profile.width_status == S5_WIDTH_LEFT_EDGE &&
               !profile.left_edge_valid, "a partial sweep estimates the observed span");
    unsigned char packet[S1_OBSTACLE_BYTES];
    TEST_CHECK("S5-U03", s1_wire_obstacle(packet, S1_OBSTACLE_DETECTED, S5_UNKNOWN_WIDTH) &&
               s1_wire_get(packet, 0, 11) == S1_WIDTH_UNAVAILABLE &&
               s1_wire_get(packet, 11, 2) == S1_OBSTACLE_DETECTED &&
               !s1_wire_obstacle(packet, S1_OBSTACLE_CLEAR, 2001),
               "obstacle detection is reported even when width is unavailable");
    S5_RANGE_GATE gate = {.range_mm = S5_UNKNOWN_RANGE, .blocked = true};
    unsigned open = s5_range_from_echo(25000);
    TEST_CHECK("S5-U04", open == S5_OPEN_RANGE && s5_range_from_echo(0) == S5_UNKNOWN_RANGE &&
               s5_range_from_echo(875) == 150 && !s5_accept_range(&gate, open) &&
               gate.blocked && s5_accept_range(&gate, open) && !gate.blocked,
               "two complete out-of-range pulses clear the startup interlock");
    TEST_CHECK("S5-U05", s5_accept_range(&gate, 100) && gate.blocked &&
               !s5_accept_range(&gate, open) && gate.range_mm == 100 &&
               s5_accept_range(&gate, S5_UNKNOWN_RANGE) && gate.blocked &&
               !s5_accept_range(&gate, open) && gate.blocked,
               "one open return cannot clear a close obstacle or a fault");
    for (unsigned i = 0; i < S5_SCAN_POINTS; ++i) s5_scan_set(&samples, i, 140);
    profile = s5_profile(&samples, 0, S5_SCAN_POINTS - 1, S5_CENTRE_INDEX, 0);
    TEST_CHECK("S5-U06", !profile.left_edge_valid && !profile.right_edge_valid &&
               profile.left_clipped && profile.right_clipped,
               "identical near readings at every angle give no confirmed exit side");
    TEST_CHECK("S5-U07", s5_classify_echo(S5_ECHO_QUIET, 0).range_mm == S5_UNKNOWN_RANGE &&
               s5_classify_echo(S5_ECHO_PULSE, 2).range_mm == S5_UNKNOWN_RANGE &&
               s5_classify_echo(S5_ECHO_HIGH, 0).range_mm == S5_UNKNOWN_RANGE,
               "silence, short pulses and pre-trigger high leave range unknown");
    S5_PING target = s5_classify_echo(S5_ECHO_PULSE, 2300);
    TEST_CHECK("S5-U08", target.range_mm == 394 && target.pulse_us == 2300 &&
               s5_classify_echo(S5_ECHO_QUIET, 0).range_mm == S5_UNKNOWN_RANGE &&
               s5_classify_echo(S5_ECHO_TIMING, 0).range_mm == S5_UNKNOWN_RANGE,
               "a valid pulse does not make a subsequent missing echo available");
    TEST_CHECK("S5-U09", s5_classify_echo(S5_ECHO_PULSE, 25000).range_mm == S5_OPEN_RANGE &&
               s5_classify_echo(S5_ECHO_PULSE, 30000).range_mm == S5_UNKNOWN_RANGE,
               "an out-of-range pulse must finish before the 30 ms deadline");
    TEST_SUMMARY("subsystem5");
}
#endif
