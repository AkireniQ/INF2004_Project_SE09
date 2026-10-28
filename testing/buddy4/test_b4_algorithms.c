/*
 * Host-side synthetic tests for Buddy 4 calibration and motion processing.
 * Build from the repository root with:
 *   cc -std=c11 -Wall -Wextra -Werror -pedantic \
 *     app_program/subsystem4/b4_imu.c \
 *     app_program/subsystem4/b4_event.c \
 *     app_program/subsystem4/b4_calibration.c \
 *     app_program/subsystem4/b4_processor.c \
 *     testing/buddy4/test_b4_algorithms.c -o /tmp/test_b4_algorithms
 */

#include <stdio.h>
#include <string.h>

#include "../../app_program/subsystem4/b4_processor.h"

static int test_failures;

#define CHECK(condition) \
	do { \
		if (!(condition)) { \
			(void)printf("FAIL %s:%d: %s\n", \
				     __FILE__, __LINE__, #condition); \
			test_failures++; \
		} \
	} while (0)

static B4_IMU_SAMPLE make_sample(uint32_t timestamp_ms,
				 int16_t x_mg,
				 int16_t y_mg,
				 int16_t z_mg,
				 int16_t mag_x,
				 int16_t mag_y)
{
	B4_IMU_SAMPLE sample = {0};

	sample.timestamp_ms = timestamp_ms;
	sample.accel_mg[0] = x_mg;
	sample.accel_mg[1] = y_mg;
	sample.accel_mg[2] = z_mg;
	sample.mag_dut[0] = mag_x;
	sample.mag_dut[1] = mag_y;
	sample.flags = B4_IMU_SAMPLE_FLAG_ACCEL_VALID
		     | B4_IMU_SAMPLE_FLAG_MAG_VALID;
	return sample;
}

static void init_processor(B4_PROCESSOR *processor)
{
	B4_PROCESSOR_CONFIG config;

	b4_processor_default_config(&config);
	config.calibration.required_samples = 10U;
	CHECK(b4_processor_init(processor, &config) ==
	      B4_CALIBRATION_COLLECTING);
}

static void calibrate_flat(B4_PROCESSOR *processor,
			   B4_PROCESSOR_OUTPUT *output,
			   uint32_t *timestamp_ms)
{
	uint8_t index;

	for (index = 0U; index < 10U; index++) {
		B4_IMU_SAMPLE sample = make_sample(*timestamp_ms,
						   0, 0, 1000, 1000, 0);

		CHECK(b4_processor_update(processor, &sample, 0U, 0, output) ==
		      ((index == 9U) ? B4_CALIBRATION_COMPLETE
				     : B4_CALIBRATION_COLLECTING));
		*timestamp_ms += 10U;
	}
	CHECK(output->calibration_ready != 0U);
	CHECK(processor->baseline_mg[2] == 1000);
}

static uint8_t output_has_event(const B4_PROCESSOR_OUTPUT *output,
				uint8_t event_type,
				uint8_t require_valid_value)
{
	uint8_t index;

	for (index = 0U; index < output->event_count; index++) {
		if ((output->events[index].type == event_type) &&
		    ((require_valid_value == 0U) ||
		     ((output->events[index].flags &
		       B4_EVENT_FLAG_VALUE_VALID) != 0U))) {
			return 1U;
		}
	}
	return 0U;
}

static void test_calibration_and_flat(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_FLAT);
	CHECK(output.tilt_active == 0U);
	CHECK(output.motion_class == B4_MOTION_STATIONARY);
	CHECK(output.event_count == 0U);
}

static void test_tilt_hysteresis(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_tilt_event = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   342, 0, 940, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		if (output_has_event(&output, B4_EVENT_TILT, 1U) != 0U) {
			saw_tilt_event = 1U;
		}
		timestamp_ms += 10U;
	}
	CHECK(saw_tilt_event != 0U);
	CHECK(output.tilt_active != 0U);
	CHECK(output.slope_cdeg > 1500);

	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	CHECK(output.tilt_active == 0U);
}

static void test_ramp_and_hump_height(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_terrain_start = 0U;
	uint8_t saw_hump = 0U;
	uint8_t saw_valid_height = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 50U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   208, 0, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_terrain_start |= output_has_event(
			&output, B4_EVENT_TERRAIN_START, 1U);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_ASCENDING);

	for (index = 0U; index < 50U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   -208, 0, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_hump |= output_has_event(&output, B4_EVENT_HUMP_DETECTED, 1U);
		saw_valid_height |= output_has_event(
			&output, B4_EVENT_HUMP_PEAK_HEIGHT, 1U);
		timestamp_ms += 10U;
	}
	CHECK(saw_terrain_start != 0U);
	CHECK(saw_hump != 0U);
	CHECK(saw_valid_height != 0U);
	CHECK(processor.peak_height_um > 0);
	CHECK(processor.peak_height_um < INT64_C(100000));

	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_FLAT);
}

static void test_ramp_event_and_end(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_ramp = 0U;
	uint8_t saw_terrain_end = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 35U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   208, 0, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 35U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_ramp |= output_has_event(&output, B4_EVENT_RAMP_DETECTED, 1U);
		saw_terrain_end |=
			output_has_event(&output, B4_EVENT_TERRAIN_END, 0U);
		timestamp_ms += 10U;
	}
	CHECK(saw_ramp != 0U);
	CHECK(saw_terrain_end != 0U);
	CHECK(output.terrain_state == B4_TERRAIN_FLAT);
}

static void test_ramp_without_odometry_has_no_height(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   208, 0, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_ASCENDING);
	CHECK(output.height_valid == 0U);
	CHECK(output.peak_height_mm == 0);
}

static void test_hump_without_odometry_marks_height_unavailable(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_unavailable_height = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   208, 0, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   -208, 0, 978, 1000, 0);
		uint8_t event_index;

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		for (event_index = 0U;
		     event_index < output.event_count; event_index++) {
			if ((output.events[event_index].type ==
			     B4_EVENT_HUMP_PEAK_HEIGHT) &&
			    ((output.events[event_index].flags &
			      B4_EVENT_FLAG_VALUE_VALID) == 0U)) {
				saw_unavailable_height = 1U;
			}
		}
		timestamp_ms += 10U;
	}
	CHECK(saw_unavailable_height != 0U);
}

static void test_vibration_and_collision(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_vibration = 0U;
	uint8_t saw_collision = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(
			timestamp_ms, ((index & 1U) != 0U) ? 400 : -400,
			0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		saw_vibration |=
			(output.motion_class == B4_MOTION_VIBRATION);
		timestamp_ms += 10U;
	}
	CHECK(saw_vibration != 0U);

	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	{
		B4_IMU_SAMPLE impact = make_sample(timestamp_ms,
						   0, 0, 3500, 1000, 0);

		(void)b4_processor_update(&processor, &impact, 0U, 0, &output);
		saw_collision =
			output_has_event(&output, B4_EVENT_COLLISION, 1U);
	}
	CHECK(saw_collision != 0U);
	CHECK(output.motion_class == B4_MOTION_IMPACT);
}

static void test_compass_turn_estimate(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	{
		B4_IMU_SAMPLE first = make_sample(timestamp_ms,
						  0, 0, 1000, 1000, 0);
		B4_IMU_SAMPLE second = make_sample(timestamp_ms + 100U,
						   0, 0, 1000, 940, 342);

		(void)b4_processor_update(&processor, &first, 0U, 0, &output);
		(void)b4_processor_update(&processor, &second, 0U, 0, &output);
		CHECK(output.turn_rate_valid != 0U);
		CHECK(output.turn_rate_mdps > 0);
		CHECK(output_has_event(&output, B4_EVENT_TURN_RATE, 1U) != 0U);
	}

}

static void test_calibration_rejects_movement(void)
{
	B4_CALIBRATION_CONFIG config = {
		6U, 850U, 1150U, 2500U
	};
	B4_CALIBRATOR calibrator;
	B4_IMU_SAMPLE sample;
	B4_CALIBRATION_STATUS status = B4_CALIBRATION_COLLECTING;
	uint8_t index;

	CHECK(b4_calibration_init(&calibrator, &config) ==
	      B4_CALIBRATION_COLLECTING);
	for (index = 0U; index < 6U; index++) {
		sample = make_sample((uint32_t)index * 10U,
				     (index & 1U) != 0U ? 300 : -300,
				     0, 954, 1000, 0);
		status = b4_calibration_add(&calibrator, &sample);
	}
	CHECK(status == B4_CALIBRATION_MOVING);
	CHECK(calibrator.ready == 0U);
	CHECK(calibrator.sample_count == 0U);
}

int main(void)
{
	test_calibration_and_flat();
	test_tilt_hysteresis();
	test_ramp_and_hump_height();
	test_ramp_event_and_end();
	test_ramp_without_odometry_has_no_height();
	test_hump_without_odometry_marks_height_unavailable();
	test_vibration_and_collision();
	test_compass_turn_estimate();
	test_calibration_rejects_movement();

	if (test_failures != 0) {
		(void)printf("%d Buddy 4 algorithm check(s) failed\n",
			     test_failures);
		return 1;
	}

	(void)printf("Buddy 4 algorithm tests passed\n");
	return 0;
}
