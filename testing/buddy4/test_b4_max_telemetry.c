/* Host test of production telemetry selection, FIFO and wire encoder.
 * cc -std=gnu11 -Wall -Wextra -Werror -DPROHIBIT_DEF_SIZE_T -D_PICO_RP2040_
 * -DCNF_SMP=1 -DTM_MQTT=1 -isystem include -isystem config
 * testing/buddy4/test_b4_max_telemetry.c app_program/subsystem1/s1_wire.c
 * app_program/subsystem4/b4_terrain_report.c -o /tmp/test_b4_max_telemetry
 */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem1/s1_telemetry.c"
#include "../../app_program/subsystem4/b4_terrain_report.h"

static VEHICLE_SNAPSHOT snapshot;
static unsigned published, publications;
unsigned s1_connection_status(void) { return 7; }
BOOL vehicle_get_snapshot(VEHICLE_SNAPSHOT *out) { *out = snapshot; return TRUE; }
void vehicle_record_telemetry_drop(void) { assert(0); }
bool motion_get_status(S2_STATUS *out) { (void)out; return false; }
bool motion_ready(void) { return false; }
int mqtt_bridge_send(unsigned route, const unsigned char *data, unsigned length)
{ (void)route; (void)data; (void)length; return E_OK; }
ER s1_publish_vehicle(unsigned state, unsigned faults)
{ (void)state; (void)faults; return E_OK; }
ER s1_publish_line(unsigned ir, unsigned barcode)
{ (void)ir; (void)barcode; return E_OK; }
ER s1_publish_obstacle(S1_OBSTACLE_STATUS status, unsigned width)
{ (void)status; (void)width; return E_OK; }
ER s1_publish_terrain(unsigned value)
{
    unsigned char bytes[S1_TERRAIN_BYTES];
    assert(s1_wire_terrain(bytes, value));
    assert(s1_wire_get(bytes, 11, 3) == S1_TERRAIN_MAX_MARKER);
    published = s1_wire_get(bytes, 0, 11);
    ++publications;
    return E_OK;
}

static void report(B4_TERRAIN_REPORT *r, unsigned now)
{
    uint16_t height, maximum;
    uint8_t phase;
    if (b4_terrain_report_next(r, now, 1, &height, &maximum, &phase)) {
        snapshot.height_mm = height;
        snapshot.max_height_mm = maximum;
        snapshot.terrain_valid = 1;
        snapshot.terrain_timestamp_ms = now;
        b4_terrain_report_commit(r, now);
    }
    snapshot.timestamp_ms = now;
    s1_publish_telemetry();
}

int main(void)
{
    B4_TERRAIN_REPORT r = {0};
    B4_PROCESSOR_OUTPUT out = {0};
    report(&r, 0);
    assert(published == S1_TERRAIN_UNAVAILABLE);
    out.hump_detected = out.height_valid = out.max_height_valid = out.roll_valid = 1;
    out.peak_height_mm = out.max_height_mm = 7;
    b4_terrain_report_observe(&r, &out, 100);
    report(&r, 100);
    assert(snapshot.height_mm == 7 && published == 7);
    out.peak_height_mm = 5;
    b4_terrain_report_observe(&r, &out, 200);
    report(&r, 1100);
    assert(snapshot.height_mm == 5 && published == 7);
    out.hump_detected = out.height_valid = 0;
    b4_terrain_report_observe(&r, &out, 1200);
    report(&r, 1200);
    assert(published == 7); /* Unavailable completed candidate preserves maximum. */
    out.hump_detected = out.height_valid = 1;
    out.peak_height_mm = out.max_height_mm = 9;
    b4_terrain_report_observe(&r, &out, 1300);
    report(&r, 1300);
    assert(snapshot.height_mm == 5 && published == 9); /* FIFO hold does not delay max. */
    report(&r, 2200);
    assert(snapshot.height_mm == 9 && published == 9 && !r.count);
    snapshot.timestamp_ms = 3301;
    s1_publish_telemetry();
    assert(published == S1_TERRAIN_UNAVAILABLE); /* Stale controller snapshot. */
    r = (B4_TERRAIN_REPORT){0};
    snapshot = (VEHICLE_SNAPSHOT){0}; /* Production runtime restarts on boot. */
    report(&r, 0);
    assert(published == S1_TERRAIN_UNAVAILABLE && publications == 8);
    puts("PASS session maximum 7/5/9, unavailable candidate, FIFO, stale data and boot reset");
    return 0;
}
