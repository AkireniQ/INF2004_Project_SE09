/*
 * cc -std=c11 -Wall -Wextra -Werror -pedantic app_program/subsystem4/b4_terrain_report.c \
 *     app_program/subsystem1/s1_wire.c testing/buddy4/test_b4_terrain_report.c \
 *     -o /tmp/test_b4_terrain_report && /tmp/test_b4_terrain_report
 */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem1/s1_wire.h"
#include "../../app_program/subsystem4/b4_terrain_report.h"
int main(void)
{
    B4_TERRAIN_REPORT r = {0};
    B4_PROCESSOR_OUTPUT out = {0};
    uint16_t height, maximum;
    uint8_t phase;
    assert(!b4_terrain_report_next(&r, 0, 1, &height, &maximum, &phase));
    out.roll_valid = 1; out.terrain_state = B4_TERRAIN_ASCENDING;
    out.hump_detected = 1; out.peak_height_mm = 12; /* invalid */
    b4_terrain_report_observe(&r, &out, 100);
    assert(r.count == 0 && !b4_terrain_report_next(&r, 100, 1, &height, &maximum, &phase));
    out.height_valid = 1;
    out.max_height_valid = 1;
    out.max_height_mm = 37;
    b4_terrain_report_observe(&r, &out, 110);
    out.peak_height_mm = 22;
    b4_terrain_report_observe(&r, &out, 120);
    assert(!b4_terrain_report_next(&r, 120, 0, &height, &maximum, &phase) && r.count == 2);
    assert(b4_terrain_report_next(&r, 120, 1, &height, &maximum, &phase));
    assert(height == 12 && maximum == 37 && phase == B4_TERRAIN_ASCENDING);
    assert(b4_terrain_report_next(&r, 130, 1, &height, &maximum, &phase) && height == 12 && r.count == 2);
    b4_terrain_report_commit(&r, 130);
    assert(r.count == 1);
    assert(b4_terrain_report_next(&r, 330, 1, &height, &maximum, &phase) && height == 12);
    out.hump_detected = 0; out.terrain_state = B4_TERRAIN_FLAT;
    b4_terrain_report_observe(&r, &out, 1100);
    assert(b4_terrain_report_next(&r, 1130, 1, &height, &maximum, &phase));
    assert(height == 22 && maximum == 37 && phase == B4_TERRAIN_FLAT);
    b4_terrain_report_commit(&r, 1130);
    assert(!r.count && b4_terrain_report_next(&r, 2201, 1, &height, &maximum, &phase));
    assert(height == 22 && maximum == 37 && phase == B4_TERRAIN_PHASE_UNAVAILABLE);

    unsigned char packet[S1_TERRAIN_BYTES];
    assert(s1_wire_terrain(packet, height));
    assert(packet[0] == 0x16 && packet[1] == 0x30);
    assert(!s1_wire_terrain(packet, B4_TERRAIN_HEIGHT_MAX_MM + 1u));
    assert(s1_wire_terrain(packet, S1_TERRAIN_UNAVAILABLE));
    assert(packet[0] == 0xff && packet[1] == 0x37);

    out.hump_detected = 1;
    for (unsigned i = 0; i < B4_TERRAIN_QUEUE_SIZE; i++) b4_terrain_report_observe(&r, &out, 2300);
    out.peak_height_mm = out.max_height_mm = 51;
    b4_terrain_report_observe(&r, &out, 2301);
    assert(r.count == B4_TERRAIN_QUEUE_SIZE && r.dropped == 1);
    assert(b4_terrain_report_next(&r, 2301, 1, &height, &maximum, &phase));
    assert(height == 22 && maximum == 51);
    puts("Buddy 4 terrain report tests passed");
    return 0;
}
