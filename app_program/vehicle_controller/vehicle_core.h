#ifndef VEHICLE_CORE_H
#define VEHICLE_CORE_H
#include <stdbool.h>
#include <stdint.h>
#include "../subsystem5/s5_avoid.h"
#define VC_LINE_MAX_AGE_MS 150u
#define VC_LINE_REACQUIRE_WAIT_MS 1000u
#define VC_SENSOR_MAX_AGE_MS 1000u
#define VC_JUNCTION_SEARCH_DEG 18u
typedef enum { VC_IDLE, VC_FOLLOW_LINE, VC_TURNING, VC_AVOIDING,
               VC_RECOVER_LINE, VC_FAULT } VC_STATE;
typedef enum { VC_REASON_NONE, VC_REASON_STOP, VC_REASON_LINE_STOP,
               VC_REASON_STALE_LINE, VC_REASON_MOTOR, VC_REASON_LINK,
               VC_REASON_OBSTACLE, VC_REASON_BARCODE_LINE } VC_REASON;
typedef enum { VC_START_READY, VC_START_STARTUP, VC_START_LINK, VC_START_MOTOR,
               VC_START_STOP_PENDING, VC_START_NO_LINE, VC_START_OLD_LINE,
               VC_START_LINE_STOP, VC_START_OBSTACLE } VC_START_BLOCK;
typedef enum { VC_NAV_NONE, VC_NAV_LEFT, VC_NAV_RIGHT, VC_NAV_UTURN } VC_NAV;
typedef enum { VC_JUNCTION_ADVANCE, VC_JUNCTION_PIVOT, VC_JUNCTION_SWEEP,
               VC_JUNCTION_SETTLE } VC_JUNCTION_PHASE;
typedef struct {
    unsigned start:1, stop:1, reset:1, ready:1, link_ok:1, line_valid:1, line_stop:1;
    unsigned obstacle_blocked:1, drive_failed:1, motor_busy:1;
    unsigned motor_faults:4, left_rpm:7, right_rpm:7;
    unsigned barcode_turn:2, pivot_in_window:1;
    uint32_t line_age_ms;
    S5_AVOID_INPUT avoidance;
} VC_INPUT;
typedef struct {
    unsigned state:3, reason:3, reposition_required:1;
    S5_AVOID avoidance;
    struct {
        uint32_t phase_at, line_at;
        unsigned phase:2, right:1, uturn:1, dark_samples:2, light_samples:2;
    } junction;
} VC_MODEL;
typedef struct {
    unsigned stop:1, left_rpm:7, right_rpm:7;
    unsigned pivot_degrees:8, pivot_right:1;
    S5_AVOID_OUTPUT maneuver;
} VC_OUTPUT;
bool vc_core_detouring(const VC_MODEL *model);
bool vc_core_running(const VC_MODEL *model);
bool vc_core_needs_line(const VC_MODEL *model);
VC_START_BLOCK vc_core_start_block(const VC_MODEL *model, const VC_INPUT *in);
void vc_core_step(VC_MODEL *model, const VC_INPUT *in, VC_OUTPUT *out);
#endif
