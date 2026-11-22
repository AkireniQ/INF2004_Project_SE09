/*
 * Host-side synthetic tests for Buddy 4 calibration and motion processing.
 * Build from the repository root with:
 *   cc -std=c11 -Wall -Wextra -Werror -pedantic \
 *     app_program/subsystem4/b4_imu.c \
 *     app_program/subsystem4/b4_event.c \
 *     app_program/subsystem4/b4_calibration.c \
 *     app_program/subsystem4/b4_processor.c \
 *     app_program/subsystem4/b4_odometry.c \
 *     app_program/subsystem4/b4_safety_dispatch.c \
 *     testing/buddy4/test_b4_algorithms.c -o /tmp/test_b4_algorithms
 */

#include <stdio.h>
#include <string.h>

#include "../../app_program/subsystem4/b4_processor.h"
#include "../../app_program/subsystem4/b4_odometry.h"
#include "../../app_program/subsystem4/b4_safety_dispatch.h"

static int test_failures;
static uint8_t motion_stop_calls;
static uint8_t vehicle_stop_calls;

static void mock_motion_stop(void)
{
	motion_stop_calls++;
}

static void mock_vehicle_stop(void)
{
	vehicle_stop_calls++;
}

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

static B4_IMU_SAMPLE make_forward_slope_sample(uint32_t timestamp_ms,
					       int16_t forward_mg)
{
	int16_t magnitude = forward_mg;
	int16_t up_mg;

	if (magnitude < 0) {
		magnitude = (int16_t)-magnitude;
	}
	if (magnitude >= 87) {
		up_mg = 996;
	} else if (magnitude >= 70) {
		up_mg = 998;
	} else if (magnitude >= 43) {
		up_mg = 999;
	} else if (magnitude >= 26) {
		up_mg = 1000;
	} else {
		up_mg = 1000;
	}
	return make_sample(timestamp_ms, 0,
			   (int16_t)-forward_mg, up_mg, 1000, 0);
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
	CHECK(processor.config.forward_axis == 1U);
	CHECK(processor.config.forward_sign == -1);
	CHECK(processor.config.up_axis == 2U);
	CHECK(processor.config.up_sign == 1);
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
						   0, -342, 940, 1000, 0);

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

static void test_shallow_hump_terrain_and_height(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_terrain_start = 0U;
	uint8_t saw_hump = 0U;
	uint8_t saw_valid_height = 0U;
	uint8_t saw_uphill_slope = 0U;
	uint8_t saw_downhill_slope = 0U;
	uint8_t saw_terrain_end = 0U;
	uint8_t index;
	static const int16_t slope_transition_mg[] = {
		0, 26, 43, 61, 70, 87
	};
	static const int16_t downhill_ramp_mg[] = {
		87, 70, 61, 43, 26, 0
	};
	uint8_t segment;

	init_processor(&processor);
	CHECK(processor.config.slope_enter_cdeg == 300U);
	CHECK(processor.config.slope_exit_cdeg == 200U);
	CHECK(processor.config.terrain_transition_samples == 5U);
	CHECK(processor.config.terrain_end_samples == 75U);
	calibrate_flat(&processor, &output, &timestamp_ms);

	for (index = 0U; index < 120U; index++) {
		int16_t noise_mg = ((index % 4U) < 2U) ? 18 : -18;
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, noise_mg);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		CHECK(output.terrain_state == B4_TERRAIN_FLAT);
		timestamp_ms += 10U;
	}

	for (segment = 0U;
	     segment < (uint8_t)(sizeof(slope_transition_mg) /
				 sizeof(slope_transition_mg[0]));
	     segment++) {
		for (index = 0U; index < 10U; index++) {
			B4_IMU_SAMPLE sample = make_forward_slope_sample(
				timestamp_ms, slope_transition_mg[segment]);

			(void)b4_processor_update(&processor, &sample, 1U, 200,
						  &output);
			saw_terrain_start |= output_has_event(
				&output, B4_EVENT_TERRAIN_START, 1U);
			timestamp_ms += 10U;
		}
	}
	for (index = 0U; index < 52U; index++) {
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, 87);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_uphill_slope |=
			(output.slope_cdeg >= 400) &&
			(output.slope_cdeg <= 600);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_ASCENDING);
	CHECK(saw_uphill_slope != 0U);

	for (index = 0U; index < 20U; index++) {
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	for (segment = 0U;
	     segment < (uint8_t)(sizeof(slope_transition_mg) /
				 sizeof(slope_transition_mg[0]));
	     segment++) {
		for (index = 0U; index < 10U; index++) {
			int16_t forward_mg =
				(int16_t)-slope_transition_mg[segment];
			B4_IMU_SAMPLE sample = make_forward_slope_sample(
				timestamp_ms, forward_mg);

			(void)b4_processor_update(&processor, &sample, 1U, 200,
						  &output);
			saw_downhill_slope |=
				(output.slope_cdeg <= -400) &&
				(output.slope_cdeg >= -600);
			saw_hump |= output_has_event(
				&output, B4_EVENT_HUMP_DETECTED, 1U);
			saw_valid_height |= output_has_event(
				&output, B4_EVENT_HUMP_PEAK_HEIGHT, 1U);
			timestamp_ms += 10U;
		}
	}
	CHECK(output.terrain_state == B4_TERRAIN_DESCENDING);
	for (index = 0U; index < 52U; index++) {
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, -87);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_hump |= output_has_event(
			&output, B4_EVENT_HUMP_DETECTED, 1U);
		saw_valid_height |= output_has_event(
			&output, B4_EVENT_HUMP_PEAK_HEIGHT, 1U);
		timestamp_ms += 10U;
	}
	for (segment = 0U;
	     segment < (uint8_t)(sizeof(downhill_ramp_mg) /
				 sizeof(downhill_ramp_mg[0]));
	     segment++) {
		for (index = 0U; index < 10U; index++) {
			int16_t forward_mg =
				(int16_t)-downhill_ramp_mg[segment];
			B4_IMU_SAMPLE sample = make_forward_slope_sample(
				timestamp_ms, forward_mg);

			(void)b4_processor_update(&processor, &sample, 1U, 200,
						  &output);
			saw_terrain_end |= output_has_event(
				&output, B4_EVENT_TERRAIN_END, 0U);
			timestamp_ms += 10U;
		}
	}
	CHECK(saw_terrain_start != 0U);
	CHECK(saw_hump != 0U);
	CHECK(saw_downhill_slope != 0U);
	CHECK(saw_valid_height != 0U);
	CHECK(saw_terrain_end != 0U);
	CHECK(output.terrain_state == B4_TERRAIN_FLAT);
	CHECK(output.max_height_mm >= 12);
	CHECK(output.max_height_mm <= 25);
}

static void test_simulated_speed_two_centimetre_hump(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_hump = 0U;
	uint8_t saw_valid_height = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);

	/* 22.5 cm at 150 mm/s is 1.5 s, or 150 samples at 100 Hz. */
	for (index = 0U; index < 150U; index++) {
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, 89);

		(void)b4_processor_update(&processor, &sample, 1U, 150,
					  &output);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_ASCENDING);
	CHECK(output.height_valid != 0U);
	CHECK(output.peak_height_mm >= 16);
	CHECK(output.peak_height_mm <= 22);

	for (index = 0U; index < 150U; index++) {
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, -89);

		(void)b4_processor_update(&processor, &sample, 1U, 150,
					  &output);
		saw_hump |= output_has_event(
			&output, B4_EVENT_HUMP_DETECTED, 1U);
		saw_valid_height |= output_has_event(
			&output, B4_EVENT_HUMP_PEAK_HEIGHT, 1U);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_DESCENDING);

	for (index = 0U; index < 80U; index++) {
		B4_IMU_SAMPLE sample =
			make_forward_slope_sample(timestamp_ms, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 150,
					  &output);
		timestamp_ms += 10U;
	}

	CHECK(saw_hump != 0U);
	CHECK(saw_valid_height != 0U);
	CHECK(output.terrain_state == B4_TERRAIN_FLAT);
	CHECK(output.max_height_mm >= 16);
	CHECK(output.max_height_mm <= 22);
}

static void test_ramp_and_hump_height(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_terrain_start = 0U;
	uint8_t saw_hump = 0U;
	uint8_t saw_valid_height = 0U;
	uint8_t saw_positive_uphill_slope = 0U;
	uint8_t saw_negative_downhill_slope = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 50U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, -208, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_positive_uphill_slope |= output.slope_cdeg >= 800;
		saw_terrain_start |= output_has_event(
			&output, B4_EVENT_TERRAIN_START, 1U);
		timestamp_ms += 10U;
	}
	CHECK(output.terrain_state == B4_TERRAIN_ASCENDING);
	CHECK(saw_positive_uphill_slope != 0U);

	for (index = 0U; index < 50U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 208, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		saw_negative_downhill_slope |= output.slope_cdeg <= -800;
		saw_hump |= output_has_event(&output, B4_EVENT_HUMP_DETECTED, 1U);
		saw_valid_height |= output_has_event(
			&output, B4_EVENT_HUMP_PEAK_HEIGHT, 1U);
		timestamp_ms += 10U;
	}
	CHECK(saw_terrain_start != 0U);
	CHECK(saw_negative_downhill_slope != 0U);
	CHECK(saw_hump != 0U);
	CHECK(saw_valid_height != 0U);
	CHECK(processor.peak_height_um > 0);
	CHECK(processor.peak_height_um < INT64_C(100000));

	for (index = 0U; index < 80U; index++) {
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
						   0, -208, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 100U; index++) {
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
						   0, -208, 978, 1000, 0);

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
						   0, -208, 978, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 208, 978, 1000, 0);
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

static void test_vibration_without_collision_detection(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_vibration = 0U;
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
		B4_IMU_SAMPLE high_acceleration = make_sample(timestamp_ms,
						   0, 0, 3500, 1000, 0);

		(void)b4_processor_update(&processor, &high_acceleration, 0U, 0,
					  &output);
	}
	CHECK(output.motion_class == B4_MOTION_DYNAMIC);
	CHECK((output.safety_flags & B4_PROCESSOR_FLAG_STOP_LATCHED) == 0U);
	CHECK(output_has_event(&output, B4_EVENT_LIFT_OR_OVERTURN, 0U) == 0U);
}

static void test_lift_detection_latches_one_stop_request(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t stop_requests = 0U;
	uint8_t lift_events = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);

	for (index = 0U; index < 2U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, 0,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_REQUESTED) == 0U);
		timestamp_ms += 10U;
	}

	for (index = 0U; index < 8U; index++) {
		B4_IMU_SAMPLE sample = (index == 0U)
			? make_sample(timestamp_ms, 0, 0, 0, 1000, 0)
			: make_sample(timestamp_ms, 0, 0, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		if ((output.safety_flags &
		     B4_PROCESSOR_FLAG_STOP_REQUESTED) != 0U) {
			stop_requests++;
		}
		lift_events = (uint8_t)(lift_events +
			output_has_event(&output, B4_EVENT_LIFT_OR_OVERTURN, 0U));
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_LATCHED) != 0U);
		CHECK((output.safety_flags &
		       (uint8_t)~B4_PROCESSOR_FLAG_VALID_MASK) == 0U);
		timestamp_ms += 10U;
	}
	CHECK(stop_requests == 1U);
	CHECK(lift_events == 1U);
}

static void test_overturn_detection_and_stop_latch(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_stop_request = 0U;
	uint8_t saw_overturn_event = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, -1000,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		saw_stop_request |=
			(output.safety_flags & B4_PROCESSOR_FLAG_STOP_REQUESTED)
			!= 0U;
		saw_overturn_event |=
			output_has_event(&output, B4_EVENT_LIFT_OR_OVERTURN, 0U);
		timestamp_ms += 10U;
	}
	CHECK(saw_stop_request != 0U);
	CHECK(saw_overturn_event != 0U);
	CHECK((output.safety_flags & B4_PROCESSOR_FLAG_STOP_LATCHED) != 0U);

	for (index = 0U; index < 10U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, 1000,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_LATCHED) != 0U);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_REQUESTED) == 0U);
		timestamp_ms += 10U;
	}
}

static void test_lift_tilt_detection(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_stop_request = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 866, 0, 500,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		saw_stop_request |=
			(output.safety_flags & B4_PROCESSOR_FLAG_STOP_REQUESTED)
			!= 0U;
		timestamp_ms += 10U;
	}
	CHECK(saw_stop_request != 0U);
	CHECK((output.safety_flags & B4_PROCESSOR_FLAG_STOP_LATCHED) != 0U);
}

static void test_driving_and_humps_do_not_trigger_safety_stop(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);

	for (index = 0U; index < 30U; index++) {
		int16_t forward_accel = ((index % 2U) != 0U) ? 450 : -450;
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0,
						   forward_accel, 1000, 1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_LATCHED) == 0U);
		timestamp_ms += 10U;
	}

	for (index = 0U; index < 2U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, 0,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_LATCHED) == 0U);
		timestamp_ms += 10U;
	}

	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, 1000,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, -208, 978,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_LATCHED) == 0U);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 208, 978,
						   1000, 0);

		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		CHECK((output.safety_flags &
		       B4_PROCESSOR_FLAG_STOP_LATCHED) == 0U);
		timestamp_ms += 10U;
	}
}

static void test_safety_stop_dispatch(void)
{
	B4_SAFETY_DISPATCH_STATUS status;

	motion_stop_calls = 0U;
	vehicle_stop_calls = 0U;
	status = b4_safety_dispatch_stop(0U, mock_motion_stop,
					mock_vehicle_stop);
	CHECK(status == B4_SAFETY_DISPATCH_NOT_REQUESTED);
	CHECK(motion_stop_calls == 0U);
	CHECK(vehicle_stop_calls == 0U);

	status = b4_safety_dispatch_stop(
		B4_PROCESSOR_FLAG_STOP_REQUESTED,
		mock_motion_stop, mock_vehicle_stop);
	CHECK(status == B4_SAFETY_DISPATCH_DISPATCHED);
	CHECK(motion_stop_calls == 1U);
	CHECK(vehicle_stop_calls == 1U);
	CHECK(b4_safety_dispatch_stop(
		      B4_PROCESSOR_FLAG_STOP_REQUESTED,
		      0, mock_vehicle_stop) ==
	      B4_SAFETY_DISPATCH_INVALID_ARGUMENT);
	CHECK(motion_stop_calls == 1U);
	CHECK(vehicle_stop_calls == 1U);
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

static void test_multiple_humps_retain_run_maximum(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	int32_t first_max;
	uint8_t saw_first_hump = 0U;
	uint8_t saw_second_hump = 0U;
	uint8_t index;

	init_processor(&processor);
	calibrate_flat(&processor, &output, &timestamp_ms);

	for (index = 0U; index < 50U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, -208, 978, 1000, 0);
		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 50U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 208, 978, 1000, 0);
		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		if (output.hump_detected != 0U) {
			saw_first_hump = 1U;
		}
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);
		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	first_max = output.max_height_mm;
	CHECK(saw_first_hump != 0U);
	CHECK(first_max > 0);

	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, -208, 978, 1000, 0);
		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 30U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 208, 978, 1000, 0);
		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		if (output.hump_detected != 0U) {
			saw_second_hump = 1U;
		}
		timestamp_ms += 10U;
	}
	for (index = 0U; index < 40U; index++) {
		B4_IMU_SAMPLE sample = make_sample(timestamp_ms,
						   0, 0, 1000, 1000, 0);
		(void)b4_processor_update(&processor, &sample, 1U, 200,
					  &output);
		timestamp_ms += 10U;
	}

	CHECK(saw_second_hump != 0U);
	CHECK(output.max_height_mm == first_max);
	CHECK(output.max_height_mm >= output.peak_height_mm);
}

static void test_forward_only_odometry(void)
{
	int32_t speed_mm_s = 0;

	CHECK(b4_odometry_validate(1U, 0U, 100U, 100U, 1U, 10U,
					   &speed_mm_s) == B4_ODOMETRY_VALID);
	CHECK(speed_mm_s > 300 && speed_mm_s < 400);
}

static void test_odometry_validity(void)
{
	int32_t speed_mm_s = 0;

	CHECK(b4_odometry_validate(1U, 0U, 100U, 100U, 1U, 10U,
				   &speed_mm_s) == B4_ODOMETRY_VALID);
	CHECK(speed_mm_s > 300 && speed_mm_s < 400);
	CHECK(b4_odometry_validate(1U, 0U, 100U, 100U, 0U, 10U,
				   &speed_mm_s) ==
		      B4_ODOMETRY_DIRECTION_UNKNOWN);
	CHECK(b4_odometry_validate(1U, 0U, 100U, 150U, 1U, 10U,
				   &speed_mm_s) == B4_ODOMETRY_TURNING);
	CHECK(b4_odometry_validate(1U, 0U, 0U, 0U, 1U, 10U,
				   &speed_mm_s) == B4_ODOMETRY_RPM_INVALID);
	CHECK(b4_odometry_validate(1U, 1U, 100U, 100U, 1U, 10U,
				   &speed_mm_s) == B4_ODOMETRY_FAULT);
	CHECK(b4_odometry_validate(1U, 0U, 100U, 100U, 1U, 101U,
				   &speed_mm_s) == B4_ODOMETRY_STALE);
	CHECK(b4_odometry_validate(2U, 0U, 100U, 100U, 1U, 10U,
				   &speed_mm_s) == B4_ODOMETRY_NOT_FORWARD);
}

int main(void)
{
	test_calibration_and_flat();
	test_tilt_hysteresis();
	test_shallow_hump_terrain_and_height();
	test_simulated_speed_two_centimetre_hump();
	test_ramp_and_hump_height();
	test_ramp_event_and_end();
	test_ramp_without_odometry_has_no_height();
	test_hump_without_odometry_marks_height_unavailable();
	test_vibration_without_collision_detection();
	test_lift_detection_latches_one_stop_request();
	test_overturn_detection_and_stop_latch();
	test_lift_tilt_detection();
	test_driving_and_humps_do_not_trigger_safety_stop();
	test_safety_stop_dispatch();
	test_compass_turn_estimate();
	test_calibration_rejects_movement();
	test_multiple_humps_retain_run_maximum();
	test_forward_only_odometry();
	test_odometry_validity();

	if (test_failures != 0) {
		(void)printf("%d Buddy 4 algorithm check(s) failed\n",
			     test_failures);
		return 1;
	}

	(void)printf("Buddy 4 algorithm tests passed\n");
	return 0;
}
