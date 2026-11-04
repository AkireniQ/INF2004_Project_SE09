#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H
#include "../app.h"
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
void vehicle_submit_line(INT l,INT c,INT r,INT left,INT right,BOOL stop,unsigned barcode);
void vehicle_invalidate_line(void);
/* Buddy 4/5 publish only REAL measurements through these copied snapshots.
 * Obstacle detection stops motion; autonomous bypass is not implemented. */
BOOL vehicle_submit_terrain(unsigned height_mm,unsigned phase);
BOOL vehicle_submit_obstacle(unsigned near_mm,unsigned width_mm,unsigned left_mm,
                             unsigned right_mm,BOOL blocked);
typedef enum { VC_START_READY, VC_START_STARTUP, VC_START_LINK, VC_START_MOTOR,
               VC_START_STOP_PENDING, VC_START_NO_LINE, VC_START_OLD_LINE,
               VC_START_LINE_STOP, VC_START_OBSTACLE } VC_START_BLOCK;
typedef struct {
    VC_STATE state; VC_REASON reason;
    uint32_t timestamp_ms,line_timestamp_ms,terrain_timestamp_ms,obstacle_timestamp_ms;
    INT l,c,r; unsigned left_rpm,right_rpm,barcode,motor_faults;
    unsigned height_mm,terrain_phase,near_mm,width_mm,clear_left_mm,clear_right_mm;
    unsigned telemetry_drops,timing_fault_gap_us,control_gap_limit_us;
    VC_START_BLOCK start_block,last_start_reject;
    BOOL motor_ready,link_ready,start_pending,stop_pending,reset_pending;
    BOOL line_valid,line_stop,terrain_valid,obstacle_valid,obstacle_blocked;
} VEHICLE_SNAPSHOT;
BOOL vehicle_get_snapshot(VEHICLE_SNAPSHOT *out);
BOOL vehicle_is_following(void);
void vehicle_record_telemetry_drop(void);
#endif
