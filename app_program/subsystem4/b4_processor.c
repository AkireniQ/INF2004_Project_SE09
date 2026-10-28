/*
 * b4_processor.c
 * Fixed-point Buddy 4 calibration and motion/terrain processing.
 */

#include "b4_processor.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#define B4_FILTER_DIVISOR             4
#define B4_DEFAULT_GRAVITY_MIN_MG     UINT16_C(750)
#define B4_DEFAULT_GRAVITY_MAX_MG     UINT16_C(1250)
#define B4_DEFAULT_TILT_ENTER_CDEG    UINT16_C(1200)
#define B4_DEFAULT_TILT_EXIT_CDEG     UINT16_C(800)
#define B4_DEFAULT_SLOPE_ENTER_CDEG   UINT16_C(800)
#define B4_DEFAULT_SLOPE_EXIT_CDEG    UINT16_C(500)
#define B4_DEFAULT_DYNAMIC_ENTER_MG   UINT16_C(120)
#define B4_DEFAULT_VIBRATION_ENTER_MG UINT16_C(250)
#define B4_DEFAULT_IMPACT_ENTER_MG    UINT16_C(1600)
#define B4_DEFAULT_MIN_MAG_NORM_DUT   UINT16_C(100)
#define B4_DEFAULT_TURN_EVENT_MDPS    UINT16_C(15000)
#define B4_DEFAULT_TRANSITION_SAMPLES UINT8_C(3)
#define B4_TURN_EVENT_COOLDOWN_MS     UINT32_C(500)
#define B4_IMPACT_REFRACTORY_MS       UINT32_C(300)

static uint64_t b4_processor_sqrt(uint64_t value)
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

static int32_t b4_processor_abs_i32(int32_t value)
{
	return (value < 0) ? -value : value;
}

/*
 * Integer atan2 approximation, centidegrees. Its maximum error is about
 * 0.4 degrees over the supported quadrants, sufficient for thresholded
 * terrain classification rather than precision metrology.
 */
static int32_t b4_processor_atan2_cdeg(int64_t y, int64_t x)
{
	uint64_t abs_y = (y < 0) ? (uint64_t)(-y) : (uint64_t)y;
	uint64_t abs_x = (x < 0) ? (uint64_t)(-x) : (uint64_t)x;
	uint64_t small;
	uint64_t large;
	uint64_t ratio_ppm;
	uint64_t angle_scale;
	int32_t angle;

	if ((abs_x == 0U) && (abs_y == 0U)) {
		return 0;
	}

	small = (abs_x < abs_y) ? abs_x : abs_y;
	large = (abs_x > abs_y) ? abs_x : abs_y;
	ratio_ppm = (small * UINT64_C(1000000)) / large;
	angle_scale = UINT64_C(4500) +
		(UINT64_C(1500) *
		 (UINT64_C(1000000) - ratio_ppm) /
		 UINT64_C(1000000));
	angle = (int32_t)((ratio_ppm * angle_scale) /
			  UINT64_C(1000000));
	if (abs_y > abs_x) {
		angle = 9000 - angle;
	}
	if (x < 0) {
		angle = 18000 - angle;
	}
	if (y < 0) {
		angle = -angle;
	}
	return angle;
}

static void b4_processor_add_event(B4_PROCESSOR_OUTPUT *output,
				   uint32_t timestamp_ms,
				   uint8_t type,
				   uint8_t value_valid,
				   int32_t value)
{
	B4_EVENT *event;

	if (output->event_count >= B4_PROCESSOR_MAX_EVENTS) {
		if (output->events_dropped < UINT8_MAX) {
			output->events_dropped++;
		}
		return;
	}

	event = &output->events[output->event_count];
	event->timestamp_ms = timestamp_ms;
	event->type = type;
	event->flags = (value_valid != 0U)
		? B4_EVENT_FLAG_VALUE_VALID : 0U;
	event->value = (value_valid != 0U) ? value : 0;
	output->event_count++;
}

static uint8_t b4_processor_config_valid(const B4_PROCESSOR_CONFIG *config)
{
	if ((config->calibration.required_samples == 0U) ||
	    (config->calibration.min_norm_mg == 0U) ||
	    (config->calibration.max_norm_mg <
	     config->calibration.min_norm_mg) ||
	    (config->calibration.max_axis_variance_mg2 == 0U) ||
	    (config->gravity_min_mg == 0U) ||
	    (config->gravity_max_mg < config->gravity_min_mg) ||
	    (config->tilt_exit_cdeg >= config->tilt_enter_cdeg) ||
	    (config->slope_exit_cdeg >= config->slope_enter_cdeg) ||
	    (config->tilt_enter_cdeg > 18000U) ||
	    (config->slope_enter_cdeg > 9000U) ||
	    (config->dynamic_enter_mg == 0U) ||
	    (config->vibration_enter_mg <= config->dynamic_enter_mg) ||
	    (config->impact_enter_mg <= config->vibration_enter_mg) ||
	    (config->min_mag_norm_dut == 0U) ||
	    (config->transition_samples == 0U) ||
	    (config->terrain_end_samples < config->transition_samples) ||
	    (config->forward_axis >= B4_IMU_AXIS_COUNT) ||
	    (config->up_axis >= B4_IMU_AXIS_COUNT) ||
	    (config->forward_axis == config->up_axis) ||
	    ((config->forward_sign != -1) &&
	     (config->forward_sign != 1)) ||
	    ((config->up_sign != -1) && (config->up_sign != 1))) {
		return 0U;
	}
	return 1U;
}

void b4_processor_default_config(B4_PROCESSOR_CONFIG *config)
{
	if (config == NULL) {
		return;
	}

	(void)memset(config, 0, sizeof(*config));
	config->calibration.required_samples =
		B4_CALIBRATION_DEFAULT_SAMPLES;
	config->calibration.min_norm_mg =
		B4_CALIBRATION_DEFAULT_MIN_NORM_MG;
	config->calibration.max_norm_mg =
		B4_CALIBRATION_DEFAULT_MAX_NORM_MG;
	config->calibration.max_axis_variance_mg2 =
		B4_CALIBRATION_DEFAULT_MAX_VAR_MG2;
	config->gravity_min_mg = B4_DEFAULT_GRAVITY_MIN_MG;
	config->gravity_max_mg = B4_DEFAULT_GRAVITY_MAX_MG;
	config->tilt_enter_cdeg = B4_DEFAULT_TILT_ENTER_CDEG;
	config->tilt_exit_cdeg = B4_DEFAULT_TILT_EXIT_CDEG;
	config->slope_enter_cdeg = B4_DEFAULT_SLOPE_ENTER_CDEG;
	config->slope_exit_cdeg = B4_DEFAULT_SLOPE_EXIT_CDEG;
	config->dynamic_enter_mg = B4_DEFAULT_DYNAMIC_ENTER_MG;
	config->vibration_enter_mg = B4_DEFAULT_VIBRATION_ENTER_MG;
	config->impact_enter_mg = B4_DEFAULT_IMPACT_ENTER_MG;
	config->min_mag_norm_dut = B4_DEFAULT_MIN_MAG_NORM_DUT;
	config->turn_event_threshold_mdps = B4_DEFAULT_TURN_EVENT_MDPS;
	config->transition_samples = B4_DEFAULT_TRANSITION_SAMPLES;
	config->terrain_end_samples = UINT8_C(15);
	config->forward_axis = 0U;
	config->up_axis = 2U;
	config->forward_sign = 1;
	config->up_sign = 1;
}

B4_CALIBRATION_STATUS b4_processor_init(
	B4_PROCESSOR *processor,
	const B4_PROCESSOR_CONFIG *config)
{
	B4_PROCESSOR_CONFIG selected;
	B4_CALIBRATION_STATUS status;

	if (processor == NULL) {
		return B4_CALIBRATION_INVALID_ARGUMENT;
	}
	if (config == NULL) {
		b4_processor_default_config(&selected);
	} else {
		selected = *config;
	}
	if (b4_processor_config_valid(&selected) == 0U) {
		return B4_CALIBRATION_INVALID_ARGUMENT;
	}

	(void)memset(processor, 0, sizeof(*processor));
	processor->config = selected;
	processor->motion_class = B4_MOTION_CALIBRATING;
	status = b4_calibration_init(&processor->calibrator,
				     &selected.calibration);
	if (status != B4_CALIBRATION_COLLECTING) {
		return status;
	}
	processor->initialized = 1U;
	return B4_CALIBRATION_COLLECTING;
}

static int32_t b4_processor_slope_cdeg(const B4_PROCESSOR *processor)
{
	uint8_t forward_axis = processor->config.forward_axis;
	uint8_t up_axis = processor->config.up_axis;
	int64_t baseline_forward =
		(int64_t)processor->baseline_mg[forward_axis] *
		processor->config.forward_sign;
	int64_t baseline_up =
		(int64_t)processor->baseline_mg[up_axis] *
		processor->config.up_sign;
	int64_t current_forward =
		(int64_t)processor->filtered_accel_mg[forward_axis] *
		processor->config.forward_sign;
	int64_t current_up =
		(int64_t)processor->filtered_accel_mg[up_axis] *
		processor->config.up_sign;
	int64_t cross = (current_forward * baseline_up) -
			(current_up * baseline_forward);
	int64_t dot = (current_forward * baseline_forward) +
		      (current_up * baseline_up);

	return b4_processor_atan2_cdeg(cross, dot);
}

static int32_t b4_processor_tilt_cdeg(const B4_PROCESSOR *processor,
				     int32_t slope_cdeg)
{
	int64_t cross[3];
	int64_t dot = 0;
	uint64_t cross_squared;
	uint64_t cross_norm;
	uint8_t axis;
	int32_t angle;

	for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
		uint8_t next = (uint8_t)((axis + 1U) % B4_IMU_AXIS_COUNT);
		uint8_t last = (uint8_t)((axis + 2U) % B4_IMU_AXIS_COUNT);

		cross[axis] =
			((int64_t)processor->baseline_mg[next] *
			 processor->filtered_accel_mg[last]) -
			((int64_t)processor->baseline_mg[last] *
			 processor->filtered_accel_mg[next]);
		dot += (int64_t)processor->baseline_mg[axis] *
		       processor->filtered_accel_mg[axis];
	}
	cross_squared = (uint64_t)(cross[0] * cross[0]) +
			(uint64_t)(cross[1] * cross[1]) +
			(uint64_t)(cross[2] * cross[2]);
	cross_norm = b4_processor_sqrt(cross_squared);
	angle = b4_processor_atan2_cdeg((int64_t)cross_norm, dot);

	return (slope_cdeg < 0) ? -angle : angle;
}

static uint16_t b4_processor_magnitude3(const int32_t vector[3])
{
	uint64_t squared =
		(uint64_t)((int64_t)vector[0] * vector[0]) +
		(uint64_t)((int64_t)vector[1] * vector[1]) +
		(uint64_t)((int64_t)vector[2] * vector[2]);
	uint64_t magnitude = b4_processor_sqrt(squared);

	return (magnitude > UINT16_MAX) ? UINT16_MAX : (uint16_t)magnitude;
}

static int32_t b4_processor_sin_q1000(int32_t angle_cdeg)
{
	int32_t angle_mrad;
	int64_t angle_squared;
	int64_t correction;

	if (angle_cdeg > 4500) {
		angle_cdeg = 4500;
	} else if (angle_cdeg < -4500) {
		angle_cdeg = -4500;
	}
	angle_mrad = (angle_cdeg * 1745) / 10000;
	angle_squared = (int64_t)angle_mrad * angle_mrad;
	correction = INT64_C(1000000) - (angle_squared / 6);
	return (int32_t)(((int64_t)angle_mrad * correction) /
			 INT64_C(1000000));
}

static int32_t b4_processor_height_mm(int64_t height_um)
{
	int64_t height_mm = height_um / 1000;

	if (height_mm > INT32_MAX) {
		return INT32_MAX;
	}
	if (height_mm < INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)height_mm;
}

static void b4_processor_emit_height(B4_PROCESSOR *processor,
				     uint32_t timestamp_ms,
				     B4_PROCESSOR_OUTPUT *output)
{
	b4_processor_add_event(output, timestamp_ms,
			       B4_EVENT_HUMP_PEAK_HEIGHT,
			       processor->height_valid,
			       processor->height_valid != 0U
			       ? b4_processor_height_mm(
					processor->peak_height_um) : 0);
}

static void b4_processor_start_terrain(B4_PROCESSOR *processor,
				       const B4_IMU_SAMPLE *sample,
				       int32_t slope_cdeg,
				       uint8_t speed_valid,
				       B4_PROCESSOR_OUTPUT *output)
{
	processor->terrain_state = B4_TERRAIN_ASCENDING;
	processor->terrain_height_um = 0;
	processor->peak_height_um = 0;
	processor->terrain_peak_slope_cdeg = slope_cdeg;
	processor->height_valid = speed_valid;
	processor->flat_count = 0U;
	b4_processor_add_event(output, sample->timestamp_ms,
			       B4_EVENT_TERRAIN_START, 1U, slope_cdeg);
}

static void b4_processor_integrate_height(B4_PROCESSOR *processor,
					  const B4_IMU_SAMPLE *sample,
					  int32_t slope_cdeg,
					  uint8_t speed_valid,
					  int32_t speed_mm_s)
{
	uint32_t elapsed_ms;
	int64_t delta_um;

	if (processor->terrain_state == B4_TERRAIN_FLAT) {
		return;
	}
	if (speed_valid == 0U) {
		processor->height_valid = 0U;
		return;
	}

	elapsed_ms = sample->timestamp_ms - processor->last_timestamp_ms;
	if ((elapsed_ms == 0U) || (elapsed_ms > 1000U)) {
		processor->height_valid = 0U;
		return;
	}
	delta_um = ((int64_t)speed_mm_s *
		    b4_processor_sin_q1000(slope_cdeg) *
		    elapsed_ms) / INT64_C(1000);
	if (((delta_um > 0) &&
	     (processor->terrain_height_um > INT64_MAX - delta_um)) ||
	    ((delta_um < 0) &&
	     (processor->terrain_height_um < INT64_MIN - delta_um))) {
		processor->height_valid = 0U;
		return;
	}
	processor->terrain_height_um += delta_um;
	if (processor->terrain_height_um > processor->peak_height_um) {
		processor->peak_height_um = processor->terrain_height_um;
	}
}

static void b4_processor_update_terrain(B4_PROCESSOR *processor,
					const B4_IMU_SAMPLE *sample,
					int32_t slope_cdeg,
					uint8_t gravity_valid,
					uint8_t speed_valid,
					int32_t speed_mm_s,
					B4_PROCESSOR_OUTPUT *output)
{
	int32_t abs_slope = b4_processor_abs_i32(slope_cdeg);
	uint8_t debounce = processor->config.transition_samples;

	if (gravity_valid == 0U) {
		processor->tilt_count = 0U;
		processor->flat_count = 0U;
		return;
	}

	if (processor->terrain_state == B4_TERRAIN_FLAT) {
		if (slope_cdeg >= processor->config.slope_enter_cdeg) {
			if (processor->slope_count < debounce) {
				processor->slope_count++;
			}
		} else {
			processor->slope_count = 0U;
		}
		if (processor->slope_count >= debounce) {
			b4_processor_start_terrain(processor, sample,
						   slope_cdeg, speed_valid,
						   output);
			processor->slope_count = 0U;
		}
		return;
	}

	b4_processor_integrate_height(processor, sample, slope_cdeg,
				      speed_valid, speed_mm_s);
	if ((processor->terrain_state == B4_TERRAIN_ASCENDING) &&
	    (slope_cdeg > processor->terrain_peak_slope_cdeg)) {
		processor->terrain_peak_slope_cdeg = slope_cdeg;
	}

	if ((processor->terrain_state == B4_TERRAIN_ASCENDING) &&
	    (slope_cdeg <= -(int32_t)processor->config.slope_enter_cdeg)) {
		if (processor->slope_count < debounce) {
			processor->slope_count++;
		}
		if (processor->slope_count >= debounce) {
			processor->terrain_state = B4_TERRAIN_DESCENDING;
			processor->slope_count = 0U;
			b4_processor_add_event(output, sample->timestamp_ms,
					       B4_EVENT_HUMP_DETECTED, 1U,
					       slope_cdeg);
			b4_processor_emit_height(processor,
						 sample->timestamp_ms,
						 output);
		}
	} else {
		processor->slope_count = 0U;
	}

	if (abs_slope <= processor->config.slope_exit_cdeg) {
		uint8_t required_flat_samples =
			(processor->terrain_state == B4_TERRAIN_ASCENDING)
			? processor->config.terrain_end_samples : debounce;

		if (processor->flat_count < required_flat_samples) {
			processor->flat_count =
				(uint8_t)(processor->flat_count + 1U);
		}
		if (processor->flat_count >= required_flat_samples) {
			if (processor->terrain_state == B4_TERRAIN_ASCENDING) {
				b4_processor_add_event(
					output, sample->timestamp_ms,
					B4_EVENT_RAMP_DETECTED, 1U,
					processor->terrain_peak_slope_cdeg);
			}
			processor->terrain_state = B4_TERRAIN_FLAT;
			processor->height_valid = 0U;
			processor->flat_count = 0U;
			processor->slope_count = 0U;
			b4_processor_add_event(output, sample->timestamp_ms,
					       B4_EVENT_TERRAIN_END, 0U, 0);
		}
	} else {
		processor->flat_count = 0U;
	}
}

static void b4_processor_update_motion(B4_PROCESSOR *processor,
				       const B4_IMU_SAMPLE *sample,
				       uint8_t gravity_valid,
				       B4_PROCESSOR_OUTPUT *output)
{
	B4_MOTION_CLASS motion = B4_MOTION_STATIONARY;
	uint32_t elapsed;

	if ((gravity_valid == 0U) ||
	    (processor->dynamic_accel_mg >
	     processor->config.dynamic_enter_mg)) {
		motion = B4_MOTION_DYNAMIC;
	}
	if (processor->dynamic_accel_mg >
	    processor->config.vibration_enter_mg) {
		if (processor->vibration_count <
		    processor->config.transition_samples) {
			processor->vibration_count++;
		}
	} else {
		processor->vibration_count = 0U;
	}
	if (processor->vibration_count >= processor->config.transition_samples) {
		motion = B4_MOTION_VIBRATION;
	}

	if (processor->dynamic_accel_mg >= processor->config.impact_enter_mg) {
		elapsed = sample->timestamp_ms -
			  processor->last_impact_timestamp_ms;
		if ((processor->impact_time_valid == 0U) ||
		    (elapsed >= B4_IMPACT_REFRACTORY_MS)) {
			processor->last_impact_timestamp_ms =
				sample->timestamp_ms;
			processor->impact_time_valid = 1U;
			motion = B4_MOTION_IMPACT;
			b4_processor_add_event(output, sample->timestamp_ms,
					       B4_EVENT_COLLISION, 1U,
					       processor->dynamic_accel_mg);
		}
	}

	if (processor->motion_class != (uint8_t)motion) {
		processor->motion_class = (uint8_t)motion;
		b4_processor_add_event(output, sample->timestamp_ms,
				       B4_EVENT_MOTION_CLASS, 1U,
				       (int32_t)motion);
	}
}

static void b4_processor_update_turn(B4_PROCESSOR *processor,
				     const B4_IMU_SAMPLE *sample,
				     B4_PROCESSOR_OUTPUT *output)
{
	int32_t x;
	int32_t y;
	uint64_t norm_squared;
	uint64_t norm;
	int32_t heading;
	uint32_t elapsed;
	int32_t delta;
	int64_t rate;

	if ((sample->flags & B4_IMU_SAMPLE_FLAG_MAG_VALID) == 0U) {
		return;
	}

	x = sample->mag_dut[0];
	y = sample->mag_dut[1];
	norm_squared = (uint64_t)((int64_t)x * x) +
		       (uint64_t)((int64_t)y * y);
	norm = b4_processor_sqrt(norm_squared);
	if (norm < processor->config.min_mag_norm_dut) {
		processor->heading_valid = 0U;
		return;
	}

	heading = b4_processor_atan2_cdeg(y, x);
	if (heading < 0) {
		heading += 36000;
	}
	if (processor->heading_valid != 0U) {
		elapsed = sample->timestamp_ms -
			  processor->last_mag_timestamp_ms;
		if ((elapsed != 0U) && (elapsed <= 1000U)) {
			delta = heading - processor->last_heading_cdeg;
			if (delta > 18000) {
				delta -= 36000;
			} else if (delta < -18000) {
				delta += 36000;
			}
			rate = ((int64_t)delta * INT64_C(10000)) / elapsed;
			if ((rate > INT32_MAX) || (rate < INT32_MIN)) {
				processor->heading_valid = 0U;
				return;
			}
			processor->filtered_turn_rate_mdps =
				(int32_t)(((int64_t)
					processor->filtered_turn_rate_mdps + rate) / 2);
			output->turn_rate_mdps =
				processor->filtered_turn_rate_mdps;
			output->turn_rate_valid = 1U;
			if (b4_processor_abs_i32(output->turn_rate_mdps) >=
			    processor->config.turn_event_threshold_mdps) {
				uint32_t since_event =
					sample->timestamp_ms -
					processor->last_turn_event_timestamp_ms;

				if ((processor->turn_event_time_valid == 0U) ||
				    (since_event >=
				     B4_TURN_EVENT_COOLDOWN_MS)) {
					b4_processor_add_event(
						output, sample->timestamp_ms,
						B4_EVENT_TURN_RATE, 1U,
						output->turn_rate_mdps);
					processor->last_turn_event_timestamp_ms =
						sample->timestamp_ms;
					processor->turn_event_time_valid = 1U;
				}
			}
		}
	}
	processor->last_heading_cdeg = heading;
	processor->last_mag_timestamp_ms = sample->timestamp_ms;
	processor->heading_valid = 1U;
}

B4_CALIBRATION_STATUS b4_processor_update(
	B4_PROCESSOR *processor,
	const B4_IMU_SAMPLE *sample,
	uint8_t speed_valid,
	int32_t speed_mm_s,
	B4_PROCESSOR_OUTPUT *output)
{
	B4_CALIBRATION_STATUS calibration_status =
		B4_CALIBRATION_COLLECTING;
	uint8_t accel_valid;
	uint8_t gravity_valid = 0U;
	int32_t raw[3];
	int32_t residual[3];
	uint8_t axis;
	uint64_t accel_squared = 0U;
	uint16_t accel_norm = 0U;
	int32_t slope_cdeg = 0;
	int32_t tilt_cdeg = 0;

	if ((processor == NULL) || (sample == NULL) || (output == NULL) ||
	    (processor->initialized == 0U) ||
	    (b4_imu_sample_is_valid(sample) == 0U) ||
	    (speed_valid > 1U)) {
		return B4_CALIBRATION_INVALID_ARGUMENT;
	}
	(void)memset(output, 0, sizeof(*output));
	accel_valid =
		((sample->flags & B4_IMU_SAMPLE_FLAG_ACCEL_VALID) != 0U)
		? 1U : 0U;

	if (processor->calibrator.ready == 0U) {
		if (accel_valid != 0U) {
			calibration_status = b4_calibration_add(
				&processor->calibrator, sample);
			if (calibration_status == B4_CALIBRATION_COMPLETE) {
				(void)b4_calibration_get_baseline(
					&processor->calibrator,
					processor->baseline_mg);
				for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
					processor->filtered_accel_mg[axis] =
						processor->baseline_mg[axis];
				}
				processor->filter_ready = 1U;
				processor->motion_class =
					B4_MOTION_STATIONARY;
				b4_processor_add_event(
					output, sample->timestamp_ms,
					B4_EVENT_CALIBRATION_READY, 0U, 0);
			}
		}
		output->motion_class = B4_MOTION_CALIBRATING;
		output->calibration_ready =
			processor->calibrator.ready;
		output->calibration_status = (uint8_t)calibration_status;
		return calibration_status;
	}

	if (accel_valid != 0U) {
		for (axis = 0U; axis < B4_IMU_AXIS_COUNT; axis++) {
			raw[axis] = sample->accel_mg[axis];
			if (processor->filter_ready == 0U) {
				processor->filtered_accel_mg[axis] = (int16_t)raw[axis];
			} else {
				int32_t delta = raw[axis] -
					processor->filtered_accel_mg[axis];
				processor->filtered_accel_mg[axis] =
					(int16_t)(processor->filtered_accel_mg[axis] +
						  delta / B4_FILTER_DIVISOR);
			}
			residual[axis] = raw[axis] -
				processor->filtered_accel_mg[axis];
			accel_squared += (uint64_t)((int64_t)
				processor->filtered_accel_mg[axis] *
				processor->filtered_accel_mg[axis]);
		}
		processor->filter_ready = 1U;
		processor->dynamic_accel_mg =
			b4_processor_magnitude3(residual);
		{
			uint64_t magnitude = b4_processor_sqrt(accel_squared);

			accel_norm = (magnitude > UINT16_MAX)
				? UINT16_MAX : (uint16_t)magnitude;
			gravity_valid =
				(accel_norm >= processor->config.gravity_min_mg) &&
				(accel_norm <= processor->config.gravity_max_mg);
		}
		slope_cdeg = b4_processor_slope_cdeg(processor);
		tilt_cdeg = b4_processor_tilt_cdeg(processor, slope_cdeg);

		if (gravity_valid != 0U) {
			if ((processor->tilt_active == 0U) &&
			    (b4_processor_abs_i32(tilt_cdeg) >=
			     processor->config.tilt_enter_cdeg)) {
				if (processor->tilt_count <
				    processor->config.transition_samples) {
					processor->tilt_count++;
				}
				if (processor->tilt_count >=
				    processor->config.transition_samples) {
					processor->tilt_active = 1U;
					processor->tilt_count = 0U;
					b4_processor_add_event(
						output, sample->timestamp_ms,
						B4_EVENT_TILT, 1U, tilt_cdeg);
				}
			} else if ((processor->tilt_active != 0U) &&
				   (b4_processor_abs_i32(tilt_cdeg) <=
				    processor->config.tilt_exit_cdeg)) {
				if (processor->tilt_release_count <
				    processor->config.transition_samples) {
					processor->tilt_release_count++;
				}
				if (processor->tilt_release_count >=
				    processor->config.transition_samples) {
					processor->tilt_active = 0U;
					processor->tilt_release_count = 0U;
				}
			} else {
				processor->tilt_count = 0U;
				processor->tilt_release_count = 0U;
			}
		} else {
			processor->tilt_count = 0U;
			processor->tilt_release_count = 0U;
		}

		b4_processor_update_terrain(processor, sample, slope_cdeg,
					    gravity_valid, speed_valid,
					    speed_mm_s, output);
		b4_processor_update_motion(processor, sample, gravity_valid,
					   output);
		processor->last_timestamp_ms = sample->timestamp_ms;
	}

	b4_processor_update_turn(processor, sample, output);
	output->calibration_ready = processor->calibrator.ready;
	output->calibration_status = (uint8_t)calibration_status;
	output->tilt_active = processor->tilt_active;
	output->terrain_state = processor->terrain_state;
	output->height_valid = processor->height_valid;
	output->peak_height_mm =
		b4_processor_height_mm(processor->peak_height_um);
	output->tilt_cdeg = (int16_t)tilt_cdeg;
	output->slope_cdeg = (int16_t)slope_cdeg;
	output->dynamic_accel_mg = processor->dynamic_accel_mg;
	output->motion_class = processor->motion_class;

	return calibration_status;
}
