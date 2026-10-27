/*
 * b4_processor.h
 * Calibration, terrain, motion, impact, and compass-rate processing.
 */

#ifndef B4_PROCESSOR_H
#define B4_PROCESSOR_H

#include <stdint.h>

#include "b4_calibration.h"
#include "b4_event.h"
#include "b4_imu.h"

#define B4_PROCESSOR_MAX_EVENTS UINT8_C(6)

typedef enum {
	B4_MOTION_CALIBRATING = 0,
	B4_MOTION_STATIONARY,
	B4_MOTION_DYNAMIC,
	B4_MOTION_VIBRATION,
	B4_MOTION_IMPACT
} B4_MOTION_CLASS;

typedef enum {
	B4_TERRAIN_FLAT = 0,
	B4_TERRAIN_ASCENDING,
	B4_TERRAIN_DESCENDING
} B4_TERRAIN_STATE;

/*
 * Defaults are conservative starting values for tuning, not values specified
 * in the project documents. Forward/up axes and signs must match the actual
 * board mounting and vehicle frame before interpreting slope/hump output.
 */
typedef struct {
	B4_CALIBRATION_CONFIG calibration;
	uint16_t gravity_min_mg;
	uint16_t gravity_max_mg;
	uint16_t tilt_enter_cdeg;
	uint16_t tilt_exit_cdeg;
	uint16_t slope_enter_cdeg;
	uint16_t slope_exit_cdeg;
	uint16_t dynamic_enter_mg;
	uint16_t vibration_enter_mg;
	uint16_t impact_enter_mg;
	uint16_t min_mag_norm_dut;
	uint16_t turn_event_threshold_mdps;
	uint8_t transition_samples;
	uint8_t terrain_end_samples;
	uint8_t forward_axis;
	uint8_t up_axis;
	int8_t forward_sign;
	int8_t up_sign;
} B4_PROCESSOR_CONFIG;

/*
 * Compact snapshot for the owning task/logger. tilt/slope are relative
 * centidegrees; dynamic_accel_mg is high-pass magnitude; turn_rate_mdps is an
 * estimated, uncompensated magnetic heading rate, never a gyroscope
 * measurement. It needs compass calibration and is susceptible to tilt and
 * motor fields. Height is an odometry-assisted estimate: the processor
 * integrates speed * sin(relative slope) over time. It is not inferred from
 * acceleration peaks and is invalid unless odometry was supplied throughout
 * the terrain event. Acceleration orientation is only a slope proxy when
 * measured acceleration is near 1 g; vehicle acceleration can confound it.
 * "Stationary" means stationary-like acceleration, not zero vehicle speed.
 */
typedef struct {
	int32_t turn_rate_mdps;
	int32_t peak_height_mm;
	int16_t tilt_cdeg;
	int16_t slope_cdeg;
	uint16_t dynamic_accel_mg;
	uint8_t calibration_ready;
	uint8_t calibration_status;
	uint8_t tilt_active;
	uint8_t terrain_state;
	uint8_t height_valid;
	uint8_t motion_class;
	uint8_t turn_rate_valid;
	uint8_t event_count;
	uint8_t events_dropped;
	B4_EVENT events[B4_PROCESSOR_MAX_EVENTS];
} B4_PROCESSOR_OUTPUT;

typedef struct {
	B4_PROCESSOR_CONFIG config;
	B4_CALIBRATOR calibrator;
	int16_t filtered_accel_mg[3];
	int16_t baseline_mg[3];
	int32_t filtered_turn_rate_mdps;
	int64_t terrain_height_um;
	int64_t peak_height_um;
	int32_t terrain_peak_slope_cdeg;
	uint32_t last_timestamp_ms;
	uint32_t last_mag_timestamp_ms;
	uint32_t last_impact_timestamp_ms;
	uint32_t last_turn_event_timestamp_ms;
	int32_t last_heading_cdeg;
	uint16_t dynamic_accel_mg;
	uint8_t initialized;
	uint8_t filter_ready;
	uint8_t tilt_active;
	uint8_t terrain_state;
	uint8_t flat_count;
	uint8_t slope_count;
	uint8_t tilt_count;
	uint8_t tilt_release_count;
	uint8_t dynamic_count;
	uint8_t vibration_count;
	uint8_t impact_time_valid;
	uint8_t turn_event_time_valid;
	uint8_t heading_valid;
	uint8_t height_valid;
	uint8_t motion_class;
} B4_PROCESSOR;

/*
 * Fill a caller-owned config with documented tuning defaults. The axis
 * mapping is only a placeholder until the IMU mounting is confirmed.
 */
void b4_processor_default_config(B4_PROCESSOR_CONFIG *config);

/* Initialize processing state and the stationary startup calibrator. */
B4_CALIBRATION_STATUS b4_processor_init(
	B4_PROCESSOR *processor,
	const B4_PROCESSOR_CONFIG *config);

/*
 * Process one sensor sample. speed_valid/speed_mm_s are the optional Buddy 2
 * odometry input. A missing speed invalidates height estimation for the
 * current terrain event; no height is inferred from acceleration peaks.
 */
B4_CALIBRATION_STATUS b4_processor_update(
	B4_PROCESSOR *processor,
	const B4_IMU_SAMPLE *sample,
	uint8_t speed_valid,
	int32_t speed_mm_s,
	B4_PROCESSOR_OUTPUT *output);

#endif /* B4_PROCESSOR_H */
