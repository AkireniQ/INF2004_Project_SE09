#ifndef S5_SERVO_CONFIG_H
#define S5_SERVO_CONFIG_H

#if __has_include("s5_servo_calibration.h")
#include "s5_servo_calibration.h"
#endif

/* Pulse widths in microseconds at the three standard bearings. */
#ifndef S5_SERVO_RIGHT_US
#define S5_SERVO_RIGHT_US 1167u
#endif
#ifndef S5_SERVO_CENTRE_US
#define S5_SERVO_CENTRE_US 1500u
#endif
#ifndef S5_SERVO_LEFT_US
#define S5_SERVO_LEFT_US 1833u
#endif
#ifndef S5_SERVO_REVERSED
#define S5_SERVO_REVERSED 0
#endif

#define S5_MIN_ANGLE 30u
#define S5_CENTRE_ANGLE 90u
#define S5_MAX_ANGLE 150u

/* Enable an outer bearing only after verifying its pulse on the mounted servo. */
#ifdef S5_SERVO_RIGHT_OUTER_US
#define S5_SCAN_MIN_ANGLE 15u
#else
#define S5_SCAN_MIN_ANGLE S5_MIN_ANGLE
#endif
#ifdef S5_SERVO_LEFT_OUTER_US
#define S5_SCAN_MAX_ANGLE 165u
#else
#define S5_SCAN_MAX_ANGLE S5_MAX_ANGLE
#endif

#endif
