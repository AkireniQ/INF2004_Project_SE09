#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"

#include <stdbool.h>
#include <stdint.h>

#define S5_PRI_SCAN                 20

#define S5_ULTRASONIC_TRIG_PIN      16u
#define S5_ULTRASONIC_ECHO_PIN      17u

#define S5_SERVO_PIN                12u
#define S5_SERVO_PWM_HZ             50u
#define S5_SERVO_PWM_DIVIDER        125u
#define S5_SERVO_MIN_PULSE_US       500u
#define S5_SERVO_MAX_PULSE_US       2500u
#define S5_SERVO_REVERSED           0

#define S5_ANGLE_MIN_DEG            0
#define S5_ANGLE_MAX_DEG            180
#define S5_ANGLE_CENTER_DEG         90

#define S5_SERVO_STEP_DEG           1
#define S5_SERVO_STEP_MS            6u
#define S5_SETTLE_MS                40u
#define S5_STARTUP_DELAY_MS         100u

#define S5_SCAN_RIGHT_DEG           30
#define S5_SCAN_LEFT_DEG            150
#define S5_COARSE_STEP_DEG          10
#define S5_COARSE_POINTS \
    (((S5_SCAN_LEFT_DEG - S5_SCAN_RIGHT_DEG) / S5_COARSE_STEP_DEG) + 1)

#define S5_OBSTACLE_THRESHOLD_CM    25.0f
#define S5_EDGE_JUMP_CM             15.0f
#define S5_MIN_CLEARANCE_CM         20.0f

typedef enum {
    S5_DIR_NONE = 0,
    S5_DIR_LEFT,
    S5_DIR_RIGHT,
    S5_DIR_BLOCKED
} s5_direction_t;

typedef struct {
    int16_t angle_deg;
    float distance_cm;
    bool valid;
} s5_sample_t;

typedef struct {
    bool obstacle_detected;
    float d_center_cm;
    float d_left_cm;
    float d_right_cm;
    int16_t left_edge_deg;
    int16_t right_edge_deg;
    bool left_edge_at_limit;
    bool right_edge_at_limit;
    float width_cm;
    float left_clearance_cm;
    float right_clearance_cm;
    s5_direction_t direction;
} s5_obstacle_profile_t;

/* The scan task sweeps continuously; the ultrasonic echo is timed while the
   servo's periodic angle updates continue running. */
IMPORT ER subsystem5_init(void);
IMPORT ER subsystem5_start(void);
IMPORT void subsystem5_test(void);

IMPORT ER s5_servo_init(void);
IMPORT ER s5_servo_set_angle(int16_t angle_deg);
IMPORT ER s5_servo_move_to(int16_t angle_deg);
IMPORT int16_t s5_servo_get_angle(void);

/* Return the most recent stored scan point nearest to the requested angle. */
IMPORT bool subsystem5_measure_at(int16_t angle_deg, s5_sample_t *out);
IMPORT bool subsystem5_check_ahead(float *distance_cm);
IMPORT size_t subsystem5_coarse_scan(s5_sample_t *buf, size_t buf_len);
IMPORT bool subsystem5_profile_obstacle(s5_obstacle_profile_t *profile);
IMPORT void subsystem5_print_profile(const s5_obstacle_profile_t *profile);
IMPORT const char *subsystem5_direction_str(s5_direction_t dir);

#endif
