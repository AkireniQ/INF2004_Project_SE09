/*
 * b4_odometry.h
 * Conservative adapter from Buddy 2 encoder RPM to Buddy 4 odometry.
 */

#ifndef B4_ODOMETRY_H
#define B4_ODOMETRY_H

#include <stdint.h>

#include "b4_imu.h"

typedef enum {
	B4_ODOMETRY_VALID = 0,
	B4_ODOMETRY_DIRECTION_UNKNOWN,
	B4_ODOMETRY_NOT_FORWARD,
	B4_ODOMETRY_FAULT,
	B4_ODOMETRY_RPM_INVALID,
	B4_ODOMETRY_TURNING,
	B4_ODOMETRY_STALE
} B4_ODOMETRY_STATUS;

/*
 * Validate public Subsystem 2 status fields without interpreting PWM as speed.
 * direction_confirmed_forward must come from a direction-aware interface.
 * It is intentionally separate because the current S2_STATUS does not expose
 * forward/reverse direction.
 */
B4_ODOMETRY_STATUS b4_odometry_validate(uint16_t mode,
	uint16_t faults,
	uint16_t left_rpm,
	uint16_t right_rpm,
	uint8_t direction_confirmed_forward,
	uint32_t elapsed_ms,
	int32_t *speed_mm_s);

#endif /* B4_ODOMETRY_H */
