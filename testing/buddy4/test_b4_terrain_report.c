/* cc -std=c11 -Wall -Wextra -Werror -pedantic app_program/subsystem4/b4_terrain_report.c testing/buddy4/test_b4_terrain_report.c -o /tmp/test_b4_terrain_report && /tmp/test_b4_terrain_report */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem4/b4_terrain_report.h"
int main(void)
{
    B4_TERRAIN_REPORT r = {0};
    B4_PROCESSOR_OUTPUT out = {0};
    uint16_t height; uint8_t phase;
    assert(!b4_terrain_report_next(&r, 0, 1, &height, &phase));
    out.roll_valid = 1; out.terrain_state = B4_TERRAIN_ASCENDING;
    out.hump_detected = 1; out.peak_height_mm = 12; /* invalid */
    b4_terrain_report_observe(&r, &out, 100);
    assert(r.count == 0 && !b4_terrain_report_next(&r, 100, 1, &height, &phase));
    out.height_valid = 1;
    b4_terrain_report_observe(&r, &out, 110);
    out.peak_height_mm = 22;
    b4_terrain_report_observe(&r, &out, 120);
    assert(!b4_terrain_report_next(&r, 120, 0, &height, &phase) && r.count == 2);
    assert(b4_terrain_report_next(&r, 120, 1, &height, &phase) && height == 12 && phase == 1);
    assert(b4_terrain_report_next(&r, 130, 1, &height, &phase) && height == 12 && r.count == 2); /* failed controller call retries */
    b4_terrain_report_commit(&r, 130);
    assert(r.count == 1);
    assert(b4_terrain_report_next(&r, 330, 1, &height, &phase) && height == 12);
    out.hump_detected = 0; out.terrain_state = B4_TERRAIN_FLAT;
    b4_terrain_report_observe(&r, &out, 1100);
    assert(b4_terrain_report_next(&r, 1130, 1, &height, &phase) && height == 22 && phase == 0);
    b4_terrain_report_commit(&r, 1130);
    assert(!r.count && b4_terrain_report_next(&r, 1131, 1, &height, &phase) && height == 22);
    assert(!b4_terrain_report_next(&r, 2201, 1, &height, &phase)); /* stale IMU */
    out.hump_detected = 1;
    for (unsigned i = 0; i < B4_TERRAIN_QUEUE_SIZE + 1; i++) b4_terrain_report_observe(&r, &out, 2300);
    assert(r.count == B4_TERRAIN_QUEUE_SIZE && r.dropped == 1);
    puts("Buddy 4 terrain report tests passed");
    return 0;
}
