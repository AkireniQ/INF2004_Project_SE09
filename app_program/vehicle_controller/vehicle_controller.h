#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H
#include "../app.h"
#include "../subsystem3/barcode.h"
#include "vehicle_core.h"
#ifndef TM_MQTT
#define TM_MQTT 0
#endif
/* No application task outside this controller normally commands S2 motors. */
ER vehicle_init(void);
ER vehicle_start_task(void);
BOOL vehicle_request_start(void); /* accepted for processing, not motion completed */
BOOL vehicle_request_reset(void); /* stopped-only, never resumes movement */
void vehicle_request_stop(void);  /* priority latch; cancels a pending START */
void vehicle_notify_feedback(void); /* motion owner or MQTT link wakeup, task context */
/* decoded is 0 for an ordinary sample, or 1..26 for a newly decoded A..Z. */
void vehicle_submit_line(INT l,INT c,INT r,INT left,INT right,BOOL stop,unsigned decoded);
void vehicle_invalidate_line(void);
BOOL vehicle_set_barcode_map(BARCODE_MAP map);
BOOL vehicle_get_barcode_map(BARCODE_MAP *map);
/* Buddy 4/5 publish measurements through these copied snapshots. */
BOOL vehicle_submit_terrain(unsigned height_mm, unsigned max_height_mm, unsigned phase);
void vehicle_enable_obstacle_monitor(void);
void vehicle_submit_obstacle_range(unsigned range_mm);
void vehicle_submit_obstacle_ping(S5_PING ping);
BOOL vehicle_obstacle_ranging_paused(void);
BOOL vehicle_obstacle_scan_request(uint32_t *request, unsigned *limit_mm);
BOOL vehicle_obstacle_scan_current(uint32_t request);
void vehicle_finish_obstacle_scan(const S5_PROFILE *profile, uint32_t request);
typedef struct {
    uint32_t timestamp_ms,line_timestamp_ms,terrain_timestamp_ms,obstacle_timestamp_ms;
    unsigned state:3, reason:3, start_block:4, last_start_reject:4;
    unsigned l:10, c:10, r:10;
    unsigned left_rpm:7, right_rpm:7, barcode:5, motor_faults:4;
    unsigned height_mm:11, max_height_mm:11, terrain_phase:2;
    unsigned near_mm:12, width_mm:11, clear_left_mm:12, clear_right_mm:12;
    unsigned obstacle_angle_offset:7, obstacle_no_echo:1;
    unsigned avoidance_phase:5, avoidance_side:2, obstacle_profile:1;
    unsigned avoidance_failure:4, avoidance_failed_phase:5, obstacle_forward_mm:12;
    unsigned motor_ready:1, link_ready:1, start_pending:1, stop_pending:1, reset_pending:1;
    unsigned line_valid:1, line_stop:1, terrain_valid:1, obstacle_valid:1, obstacle_blocked:1;
    unsigned telemetry_drops,timing_fault_gap_us,control_gap_limit_us;
} VEHICLE_SNAPSHOT;
BOOL vehicle_get_snapshot(VEHICLE_SNAPSHOT *out);
BOOL vehicle_is_following(void);
void vehicle_record_telemetry_drop(void);
#endif
