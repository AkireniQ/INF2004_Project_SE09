#ifndef S5_SERVO_CALIBRATION_H
#define S5_SERVO_CALIBRATION_H

/* Copy to s5_servo_calibration.h and set pulses verified on the car. */
#define S5_SERVO_RIGHT_US 1167u  /* 30 degrees, 60 right of forward */
#define S5_SERVO_CENTRE_US 1500u /* 90 degrees, straight ahead */
#define S5_SERVO_LEFT_US 1833u   /* 150 degrees, 60 left of forward */
#define S5_SERVO_REVERSED 0     /* Set to 1 if the mounted servo sweeps the opposite way. */

/* Optional. Define only after verifying the bearing and free travel. */
/* #define S5_SERVO_RIGHT_OUTER_US pulse_in_us */ /* 15 degrees, 75 right */
/* #define S5_SERVO_LEFT_OUTER_US pulse_in_us */  /* 165 degrees, 75 left */

#endif
