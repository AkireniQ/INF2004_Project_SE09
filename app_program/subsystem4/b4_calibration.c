/*
 * b4_calibration.c
 * Stationary accelerometer calibration for Buddy 4.
 */

#include "b4_calibration.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static uint64_t b4_calibration_sqrt(uint64_t value)
{
	uint64_t result = 0U;
	uint64_t bit = UINT64_C(1) << 62;

	while (bit > value) {
		bit >>= 2;
	}

	while (bit != 0U) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}
		bit >>= 2;
	}

	return result;
}

static void b4_calibration_clear_collection(B4_CALIBRATOR *calibrator)
{
	uint8_t axis;

	calibrator->sample_count = 0U;
	for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
		calibrator->sum_mg[axis] = 0;
		calibrator->sum_squares_mg2[axis] = 0U;
	}
}

B4_CALIBRATION_STATUS b4_calibration_init(
	B4_CALIBRATOR *calibrator,
	const B4_CALIBRATION_CONFIG *config)
{
	B4_CALIBRATION_CONFIG selected;

	if (calibrator == NULL) {
		return B4_CALIBRATION_INVALID_ARGUMENT;
	}

	if (config == NULL) {
		selected.required_samples = B4_CALIBRATION_DEFAULT_SAMPLES;
		selected.min_norm_mg = B4_CALIBRATION_DEFAULT_MIN_NORM_MG;
		selected.max_norm_mg = B4_CALIBRATION_DEFAULT_MAX_NORM_MG;
		selected.max_axis_variance_mg2 =
			B4_CALIBRATION_DEFAULT_MAX_VAR_MG2;
	} else {
		selected = *config;
	}

	if ((selected.required_samples == 0U) ||
	    (selected.min_norm_mg == 0U) ||
	    (selected.max_norm_mg < selected.min_norm_mg) ||
	    (selected.max_axis_variance_mg2 == 0U)) {
		return B4_CALIBRATION_INVALID_ARGUMENT;
	}

	(void)memset(calibrator, 0, sizeof(*calibrator));
	calibrator->config = selected;
	return B4_CALIBRATION_COLLECTING;
}

B4_CALIBRATION_STATUS b4_calibration_add(
	B4_CALIBRATOR *calibrator,
	const B4_IMU_SAMPLE *sample)
{
	uint8_t axis;
	uint16_t next_count;

	if ((calibrator == NULL) || (sample == NULL) ||
	    (calibrator->config.required_samples == 0U)) {
		return B4_CALIBRATION_INVALID_ARGUMENT;
	}
	if (calibrator->ready != 0U) {
		return B4_CALIBRATION_COMPLETE;
	}
	if ((sample->flags & B4_IMU_SAMPLE_FLAG_ACCEL_VALID) == 0U) {
		return B4_CALIBRATION_COLLECTING;
	}

	next_count = (uint16_t)(calibrator->sample_count + 1U);
	for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
		int32_t value = sample->accel_mg[axis];

		calibrator->sum_mg[axis] += value;
		calibrator->sum_squares_mg2[axis] +=
			(uint64_t)((int64_t)value * value);
	}
	calibrator->sample_count = next_count;

	if (next_count < calibrator->config.required_samples) {
		return B4_CALIBRATION_COLLECTING;
	}

	{
		int64_t mean[3];
		uint64_t norm_squared = 0U;
		uint64_t count = calibrator->config.required_samples;

		for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
			uint64_t mean_squared;
			uint64_t average_square =
				calibrator->sum_squares_mg2[axis] / count;
			uint64_t variance;

			mean[axis] = calibrator->sum_mg[axis] / (int64_t)count;
			mean_squared = (uint64_t)(mean[axis] * mean[axis]);
			variance = (average_square > mean_squared)
				? average_square - mean_squared : 0U;
			if (variance >
			    calibrator->config.max_axis_variance_mg2) {
				b4_calibration_clear_collection(calibrator);
				return B4_CALIBRATION_MOVING;
			}
			norm_squared += mean_squared;
		}

		{
			uint64_t norm_mg = b4_calibration_sqrt(norm_squared);

			if ((norm_mg < calibrator->config.min_norm_mg) ||
			    (norm_mg > calibrator->config.max_norm_mg)) {
				b4_calibration_clear_collection(calibrator);
				return B4_CALIBRATION_BAD_GRAVITY;
			}
		}

		for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
			if ((mean[axis] < INT16_MIN) ||
			    (mean[axis] > INT16_MAX)) {
				b4_calibration_clear_collection(calibrator);
				return B4_CALIBRATION_BAD_GRAVITY;
			}
			calibrator->baseline_mg[axis] = (int16_t)mean[axis];
		}
	}

	calibrator->ready = 1U;
	return B4_CALIBRATION_COMPLETE;
}

uint8_t b4_calibration_get_baseline(
	const B4_CALIBRATOR *calibrator,
	int16_t baseline_mg[3])
{
	uint8_t axis;

	if ((calibrator == NULL) || (baseline_mg == NULL) ||
	    (calibrator->ready == 0U)) {
		return 0U;
	}

	for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
		baseline_mg[axis] = calibrator->baseline_mg[axis];
	}
	return 1U;
}
