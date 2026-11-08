/*
 * b4_odometry.c
 * Encoder-derived speed validation for Buddy 4 terrain height estimation.
 */

#include <stddef.h>

#include "b4_odometry.h"
#include "../subsystem2/motion_config.h"

#define B4_ODOMETRY_VELOCITY_MODE      UINT16_C(1)
#define B4_ODOMETRY_MIN_RPM            UINT16_C(10)
#define B4_ODOMETRY_MAX_RPM            UINT16_C(300)
#define B4_ODOMETRY_STRAIGHT_LIMIT_PCT UINT16_C(10)
#define B4_ODOMETRY_MIN_ELAPSED_MS     UINT32_C(1)
#define B4_ODOMETRY_MAX_ELAPSED_MS     UINT32_C(100)

static uint16_t b4_odometry_abs_diff(uint16_t left, uint16_t right)
{
	return (left >= right) ? (uint16_t)(left - right)
		               : (uint16_t)(right - left);
}

B4_ODOMETRY_STATUS b4_odometry_validate(uint16_t mode,
	uint16_t faults,
	uint16_t left_rpm,
	uint16_t right_rpm,
	uint8_t direction_confirmed_forward,
	uint32_t elapsed_ms,
	int32_t *speed_mm_s)
{
	uint32_t average_rpm;
	uint32_t circumference_um;
	uint32_t larger_rpm;
	uint16_t difference;

	if (speed_mm_s == NULL) {
		return B4_ODOMETRY_RPM_INVALID;
	}
	*speed_mm_s = 0;

	if (elapsed_ms < B4_ODOMETRY_MIN_ELAPSED_MS ||
	    elapsed_ms > B4_ODOMETRY_MAX_ELAPSED_MS) {
		return B4_ODOMETRY_STALE;
	}
	if (direction_confirmed_forward == 0U) {
		return B4_ODOMETRY_DIRECTION_UNKNOWN;
	}
	if (mode != B4_ODOMETRY_VELOCITY_MODE) {
		return B4_ODOMETRY_NOT_FORWARD;
	}
	if (faults != 0U) {
		return B4_ODOMETRY_FAULT;
	}
	if ((left_rpm < B4_ODOMETRY_MIN_RPM) ||
	    (right_rpm < B4_ODOMETRY_MIN_RPM) ||
	    (left_rpm > B4_ODOMETRY_MAX_RPM) ||
	    (right_rpm > B4_ODOMETRY_MAX_RPM)) {
		return B4_ODOMETRY_RPM_INVALID;
	}

	difference = b4_odometry_abs_diff(left_rpm, right_rpm);
	larger_rpm = (left_rpm >= right_rpm) ? left_rpm : right_rpm;
	if ((((uint32_t)difference * UINT32_C(100)) / larger_rpm) >
	    B4_ODOMETRY_STRAIGHT_LIMIT_PCT) {
		return B4_ODOMETRY_TURNING;
	}

	average_rpm = ((uint32_t)left_rpm + (uint32_t)right_rpm) / 2U;
	circumference_um = (uint32_t)(((uint64_t)WHEEL_DIAMETER_MM *
				       UINT64_C(3141593)) / UINT64_C(1000));
	*speed_mm_s = (int32_t)((UINT64_C(1) * average_rpm *
				 circumference_um) /
				 UINT64_C(60000));
	return B4_ODOMETRY_VALID;
}
