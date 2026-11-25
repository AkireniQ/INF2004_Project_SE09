#ifndef LINE_STEERING_H
#define LINE_STEERING_H

#include <stdbool.h>
#include <stdint.h>

/* Wheel targets use the team's even-RPM Subsystem 2 API. */
#define S3_CRUISE_RPM 12
#define S3_EDGE_DARKNESS 160
#define S3_MAX_OUTER_RPM 14
#define S3_CORRECTION_STEP_RPM 2
#define S3_CURVE_CONFIRM_SAMPLES 8u /* 80 ms of consistent edge evidence */
#define S3_BLIND_CURVE_LIMIT 160u /* 1.6 s at one sample every 10 ms */
#define S3_CURVE_EXIT_SAMPLES 36u /* 360 ms, including the steering ramp down */

typedef struct {
    int16_t filtered_error;
    int8_t correction_rpm;
    int8_t candidate_side;
    int8_t curve_side;
    int8_t last_steer_side;
    uint8_t candidate_samples;
    uint8_t strong_samples;
    uint8_t light_samples;
    uint8_t centered_samples;
    uint8_t neutral_samples;
    uint8_t settle_samples;
    bool sustained_curve;
    bool slew_hold;
    bool reading_barcode;
} LF_STATE;

typedef struct {
    int left;
    int right;
    bool stop;
} LF_CMD;

void line_decide(int left, int center, int right, LF_STATE *state, LF_CMD *command);

#endif
