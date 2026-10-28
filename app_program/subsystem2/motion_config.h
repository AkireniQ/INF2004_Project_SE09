#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H
#define S2_PWM_DIV16 100u /* 6.25 in RP2040 8.4 format; 20kHz at 125MHz. */
#define LEFT_IN_A 8u
#define LEFT_IN_B 9u
#define RIGHT_IN_A 10u
#define RIGHT_IN_B 11u
#define LEFT_ENC_A 2u
#define LEFT_ENC_B 3u
#define RIGHT_ENC_A 0u
#define RIGHT_ENC_B 1u
/* Check physical direction with wheels raised. Flip the relevant value if needed. */
#define LEFT_MOTOR_INVERT 0
#define RIGHT_MOTOR_INVERT 0
#define WHEEL_DIAMETER_MM 67u
#define EFFECTIVE_TRACK_MM 108u
#define LEFT_CPR_X10 6363u
#define RIGHT_CPR_X10 6330u
#define MAX_RPM 240u /* Initial observed-speed reference, NOT a proven bound. */
#define MAX_CONTROL_GAP_US 11000u /* Exceeding this while moving stops the test. */
#define MAX_DISTANCE_MM 2000u
#define STALL_US 500000u
#define MOVE_TIMEOUT_US 30000000u
/* Integer fixed-point PID: percent PWM = feedforward + P + I + D.
   Preliminary gains only. Measurement is filtered; all must be tuned on hardware. */
#define KP_NUM 12
#define KP_DEN 100
#define KI_DEN 1000 /* Existing saved value; copy your final tuned value here if different. */
#define KD_NUM 0 /* Derivative supported but disabled initially to avoid quantised noise. */
#define KD_DEN 100
#define INTEGRAL_LIMIT 2000

/* TODO TEAM: replace these defaults after agreeing the scheduling/core allocation.
   Priority 14 retains the provided TEAM.md band; it is NOT a claim of highest priority. */
#define S2_MOTION_PRIORITY 14
#define S2_TASK_AFFINITY TP_PRC2
#define S2_PERIOD_MS 10u
#define S2_STARTUP_DELAY_MS 5000u
/* TODO TEAM: set 0 and call s2_encoder_dispatch() from shared IRQ13 on core 1. */
#define S2_OWNS_GPIO_CALLBACK 1
/* TODO TEAM: adjust only after measuring maximum pulse rate and scheduling delays. */
#define S2_QUEUE_LENGTH 8u
#define S2_COMMAND_TTL_MS 250u
#define S2_COMMAND_TIMEOUT_MS 1000u
#define S2_DIRECTION_SETTLE_US 200000u

#endif /* MOTION_CONFIG_H */
