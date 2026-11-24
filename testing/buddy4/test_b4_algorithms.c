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

static void test_ramp_event_and_end(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	uint8_t saw_ramp = 0U;
	uint8_t saw_terrain_end = 0U;
	uint8_t index;

	init_processor(&processor);
	processor.config.effective_lateral_distance_mm = 120U;
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
	CHECK(!output.max_height_valid && !output.hump_detected);
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

static void test_safety_rearm(void)
{
	B4_PROCESSOR processor;
	B4_PROCESSOR_OUTPUT output;
	uint32_t timestamp_ms = 0U;
	init_processor(&processor);
	CHECK(!b4_processor_rearm(&processor, timestamp_ms));
	calibrate_flat(&processor, &output, &timestamp_ms);
	for (unsigned cycle = 0; cycle < 2; cycle++) {
		unsigned requests = 0;
		for (unsigned i = 0; i < 30; i++) {
			B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, -1000, 1000, 0);
			(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
			requests += !!(output.safety_flags & B4_PROCESSOR_FLAG_STOP_REQUESTED);
			timestamp_ms += 10U;
		}
		CHECK(requests == 1);
		CHECK(!b4_processor_rearm(&processor, timestamp_ms));
		CHECK(processor.stop_latched);
		for (unsigned i = 0; i < 40; i++) {
			B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, 1000, 1000, 0);
			(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
			timestamp_ms += 10U;
		}
		CHECK(processor.stop_latched); /* Recovery alone must not rearm. */
		CHECK(b4_processor_restart_ready(&processor, timestamp_ms));
		CHECK(!b4_processor_rearm(&processor, timestamp_ms + 101U));
		CHECK(processor.stop_latched);
		CHECK(b4_processor_rearm(&processor, timestamp_ms));
		CHECK(!processor.stop_latched);
	}
	B4_IMU_SAMPLE sample = make_sample(timestamp_ms, 0, 0, 0, 1000, 0);
	(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
	CHECK(!b4_processor_rearm(&processor, timestamp_ms)); /* First low-g sample blocks START. */
	sample.timestamp_ms += 200U;
	sample.flags = B4_IMU_SAMPLE_FLAG_MAG_VALID;
	(void)b4_processor_update(&processor, &sample, 0U, 0, &output);
	CHECK(!b4_processor_rearm(&processor, sample.timestamp_ms));
}

static void roll_samples(B4_PROCESSOR *p, B4_PROCESSOR_OUTPUT *out,
                         uint32_t *t, int16_t x, int16_t z, unsigned count,
                         unsigned *events, unsigned *valid_events)
{
    for (unsigned i = 0; i < count; ++i) {
        B4_IMU_SAMPLE sample = make_sample(*t, x, 0, z, 1000, 0);
        (void)b4_processor_update(p, &sample, 0U, 0, out);
        *events += out->hump_detected;
        *valid_events += output_has_event(out, B4_EVENT_HUMP_PEAK_HEIGHT, 1U);
        *t += 10;
    }
}

static void test_right_wheel_humps(void)
{
    B4_PROCESSOR p;
    B4_PROCESSOR_OUTPUT out;
    B4_PROCESSOR_CONFIG config;
    uint32_t t = 0;
    unsigned events = 0, valid = 0;
    b4_processor_default_config(&config);
    CHECK(config.effective_lateral_distance_mm == 111U);
    config.effective_lateral_distance_mm = 120U; /* synthetic only */
    config.calibration.required_samples = 10U;
    CHECK(b4_processor_init(&p, &config) == B4_CALIBRATION_COLLECTING);
    calibrate_flat(&p, &out, &t);
    roll_samples(&p, &out, &t, 0, 1000, 10, &events, &valid);
    roll_samples(&p, &out, &t, 174, 985, 30, &events, &valid);
    CHECK(out.roll_valid && out.roll_cdeg < -900 && events == 0);
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 0 && !out.max_height_valid);
    roll_samples(&p, &out, &t, -174, 985, 30, &events, &valid);
    CHECK(out.roll_valid && out.roll_cdeg > 900 && out.slope_cdeg == 0);
    CHECK(out.hump_state == B4_HUMP_RIGHT_RISE && events == 0);
    CHECK(!out.max_height_valid); /* An unfinished rise must not count. */
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 1 && valid == 1 && out.max_height_valid);
    CHECK(out.max_height_mm >= 19 && out.max_height_mm <= 23);
    int first = out.max_height_mm;
    roll_samples(&p, &out, &t, -87, 996, 30, &events, &valid);
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 2 && valid == 2 && out.max_height_mm == first);
    roll_samples(&p, &out, &t, -342, 940, 30, &events, &valid);
    CHECK(out.max_height_mm == first); /* no update before return to level */
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 3 && valid == 3 && out.max_height_mm > first);
    int maximum = out.max_height_mm;
    roll_samples(&p, &out, &t, -500, 866, 30, &events, &valid);
    B4_IMU_SAMPLE bad = make_sample(t, -500, 0, 866, 1000, 0);
    bad.flags |= B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN;
    (void)b4_processor_update(&p, &bad, 0U, 0, &out); t += 10;
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 4 && valid == 3 && out.max_height_mm == maximum);
    roll_samples(&p, &out, &t, -500, 866, 30, &events, &valid);
    t += 200; /* Missing acceleration history. */
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 5 && valid == 3 && out.max_height_mm == maximum);
    roll_samples(&p, &out, &t, -500, 866, 30, &events, &valid);
    b4_processor_invalidate_hump(&p); /* I2C read failure */
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 6 && valid == 3 && out.max_height_mm == maximum);
    roll_samples(&p, &out, &t, -500, 866, 30, &events, &valid);
    bad = make_sample(t, 0, 0, 0, 1000, 0);
    (void)b4_processor_update(&p, &bad, 0U, 0, &out); t += 10;
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 7 && valid == 3 && out.max_height_mm == maximum);
}

static void test_longitudinal_reversal(void)
{
    B4_PROCESSOR p;
    B4_PROCESSOR_OUTPUT out;
    uint32_t t = 0;
    unsigned humps = 0;
    init_processor(&p);
    p.config.effective_lateral_distance_mm = 120U;
    calibrate_flat(&p, &out, &t);
    for (unsigned phase = 0; phase < 3; phase++) {
        int16_t y = phase == 0 ? -174 : phase == 1 ? 174 : 0;
        int16_t z = phase == 2 ? 1000 : 985;
        for (unsigned i = 0; i < 35; i++) {
            B4_IMU_SAMPLE sample = make_sample(t, 0, y, z, 1000, 0);
            (void)b4_processor_update(&p, &sample, 1U, 200, &out);
            humps += out.hump_detected;
            t += 10;
        }
        if (phase == 0) CHECK(out.terrain_state == B4_TERRAIN_ASCENDING && out.slope_cdeg > 900);
        if (phase == 1) CHECK(out.terrain_state == B4_TERRAIN_DESCENDING && out.slope_cdeg < -900);
    }
    CHECK(out.terrain_state == B4_TERRAIN_FLAT && humps == 0 && !out.max_height_valid);
}

static void test_unconfigured_roll_height(void)
{
    B4_PROCESSOR p;
    B4_PROCESSOR_OUTPUT out;
    uint32_t t = 0;
    unsigned events = 0, valid = 0;
    init_processor(&p);
    p.config.effective_lateral_distance_mm = 0U;
    calibrate_flat(&p, &out, &t);
    roll_samples(&p, &out, &t, 0, 1000, 10, &events, &valid);
    roll_samples(&p, &out, &t, -174, 985, 30, &events, &valid);
    roll_samples(&p, &out, &t, 0, 1000, 30, &events, &valid);
    CHECK(events == 1 && valid == 0 && !out.max_height_valid && !out.height_valid);
}

int main(void)
{
	test_right_wheel_humps();
	test_unconfigured_roll_height();
	test_longitudinal_reversal();
	test_safety_rearm();
	test_calibration_and_flat();
	test_tilt_hysteresis();
	test_ramp_event_and_end();
	test_ramp_without_odometry_has_no_height();
	test_vibration_without_collision_detection();
	test_lift_detection_latches_one_stop_request();
	test_overturn_detection_and_stop_latch();
	test_lift_tilt_detection();
	test_driving_and_humps_do_not_trigger_safety_stop();
	test_safety_stop_dispatch();
	test_compass_turn_estimate();
	test_calibration_rejects_movement();
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
