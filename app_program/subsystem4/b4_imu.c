/*
 * b4_imu.c
 * Model-independent IMU sample validation and source dispatch.
 */

#include "b4_imu.h"

uint8_t b4_imu_sample_is_valid(const B4_IMU_SAMPLE *sample)
{
	if (sample == 0U) {
		return 0U;
	}

	if ((sample->flags & (uint8_t)~B4_IMU_SAMPLE_FLAG_VALID_MASK) != 0U) {
		return 0U;
	}

	if (((sample->flags & B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN) != 0U) &&
	    ((sample->flags & B4_IMU_SAMPLE_FLAG_ACCEL_VALID) == 0U)) {
		return 0U;
	}

	return (uint8_t)((sample->flags &
			  (B4_IMU_SAMPLE_FLAG_ACCEL_VALID |
			   B4_IMU_SAMPLE_FLAG_MAG_VALID)) != 0U);
}

B4_IMU_STATUS b4_imu_read(const B4_IMU_SOURCE *source,
			  B4_IMU_SAMPLE *sample_out)
{
	B4_IMU_SAMPLE sample = {0};
	B4_IMU_STATUS status;

	if ((source == 0U) || (source->read == 0U) || (sample_out == 0U)) {
		return B4_IMU_STATUS_INVALID_ARGUMENT;
	}

	status = source->read(source->context, &sample);
	if (status != B4_IMU_STATUS_OK) {
		return status;
	}

	if (b4_imu_sample_is_valid(&sample) == 0U) {
		return B4_IMU_STATUS_INVALID_SAMPLE;
	}

	*sample_out = sample;
	return B4_IMU_STATUS_OK;
}
