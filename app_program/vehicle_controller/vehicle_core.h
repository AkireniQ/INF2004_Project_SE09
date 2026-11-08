#ifndef VEHICLE_CORE_H
#define VEHICLE_CORE_H
#include <stdbool.h>
#include <stdint.h>
#define VC_LINE_MAX_AGE_MS 150u
#define VC_SENSOR_MAX_AGE_MS 1000u
typedef enum { VC_IDLE, VC_FOLLOW_LINE, VC_TURNING, VC_AVOIDING,
               VC_RECOVER_LINE, VC_FAULT } VC_STATE;
typedef enum { VC_REASON_NONE, VC_REASON_STOP, VC_REASON_LINE_STOP,
               VC_REASON_STALE_LINE, VC_REASON_MOTOR, VC_REASON_LINK,
               VC_REASON_OBSTACLE } VC_REASON;
typedef struct {
    bool start, stop, ready, link_required, link_ok, line_valid, line_stop;
    bool obstacle_blocked;
    uint32_t line_age_ms;
    unsigned motor_faults, left_rpm, right_rpm;
} VC_INPUT;
typedef struct { VC_STATE state; VC_REASON reason; bool running; } VC_MODEL;
typedef struct { bool stop; unsigned left_rpm, right_rpm; } VC_OUTPUT;
void vc_core_step(VC_MODEL *model, const VC_INPUT *in, VC_OUTPUT *out);
#endif
