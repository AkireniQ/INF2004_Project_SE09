/*
 * b4_processor.h
 * Calibration, terrain, motion, lift/overturn, and compass-rate processing.
 */

#ifndef B4_PROCESSOR_H
#define B4_PROCESSOR_H

#include <stdint.h>

#include "b4_calibration.h"
#include "b4_event.h"
#include "b4_imu.h"

#define B4_PROCESSOR_MAX_EVENTS UINT8_C(6)
#define B4_PROCESSOR_FLAG_STOP_LATCHED UINT8_C(0x01)
#define B4_PROCESSOR_FLAG_STOP_REQUESTED UINT8_C(0x02)
#define B4_PROCESSOR_FLAG_VALID_MASK \
	(B4_PROCESSOR_FLAG_STOP_LATCHED | B4_PROCESSOR_FLAG_STOP_REQUESTED)

typedef enum {
	B4_MOTION_CALIBRATING = 0,
	B4_MOTION_STATIONARY,
	B4_MOTION_DYNAMIC,
	B4_MOTION_VIBRATION
} B4_MOTION_CLASS;

typedef enum {
	B4_TERRAIN_FLAT = 0,
	B4_TERRAIN_ASCENDING,
	B4_TERRAIN_DESCENDING
} B4_TERRAIN_STATE;

/*
 * Defaults are conservative starting values for tuning, not values specified
 * in the project documents. The current GY-511 mounting uses sensor -Y as
 * forward and +Z as up; change the axis/sign mapping if the board is remounted.
 */
/* Wheel-centre spacing is an initial geometric estimate, not a validated calibration. */
#ifndef B4_EFFECTIVE_LATERAL_DISTANCE_MM
#define B4_EFFECTIVE_LATERAL_DISTANCE_MM 111
#endif
#if B4_EFFECTIVE_LATERAL_DISTANCE_MM > 65535 || B4_EFFECTIVE_LATERAL_DISTANCE_MM < 0
#error "B4 effective lateral distance must fit in uint16_t"
#endif

typedef enum { B4_HUMP_LEVEL = 0, B4_HUMP_RIGHT_RISE } B4_HUMP_STATE;

typedef struct {
	B4_CALIBRATION_CONFIG calibration;
	uint16_t effective_lateral_distance_mm;
	uint16_t roll_enter_cdeg;
	uint16_t roll_exit_cdeg;
	uint16_t gravity_min_mg;
	uint16_t gravity_max_mg;
	uint16_t tilt_enter_cdeg;
	uint16_t tilt_exit_cdeg;
	uint16_t slope_enter_cdeg;
	uint16_t slope_exit_cdeg;
	uint16_t dynamic_enter_mg;
	uint16_t vibration_enter_mg;
	uint16_t min_mag_norm_dut;
	uint16_t turn_event_threshold_mdps;
	uint8_t transition_samples;
	/* Terrain-only debounce; shared motion/safety debounce is unchanged. */
	uint8_t terrain_transition_samples;
	/* Samples near level tolerated before ending an ascending terrain event. */
	uint8_t terrain_end_samples;
	uint8_t left_axis;
	int8_t left_sign;
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
 * motor fields. Positive roll means right side up relative to startup.
 * Hump height is peak lateral separation estimated as distance*sin(roll),
 * not the directly measured hump height. The default 111 mm is the measured
 * wheel-centre spacing, not a validated height calibration; zero disables height. Measurements
 * are invalidated by bad gravity, missing/overrun samples or safety events.
 * Acceleration and chassis dynamics can confound tilt and roll.
 * "Stationary" means stationary-like acceleration, not zero vehicle speed.
 */
typedef struct {
	int32_t turn_rate_mdps;
	int32_t peak_height_mm;
	int32_t max_height_mm;
	int16_t tilt_cdeg;
	int16_t slope_cdeg;
	int16_t roll_cdeg;
	uint16_t dynamic_accel_mg;
	uint8_t calibration_ready;
	uint8_t calibration_status;
	uint8_t tilt_active;
	uint8_t hump_detected;
	uint8_t terrain_state;
	uint8_t height_valid; /* most recent completed hump has a valid height */
	uint8_t max_height_valid;
	uint8_t roll_valid;
	uint8_t hump_state;
	uint8_t motion_class;
	uint8_t turn_rate_valid;
	uint8_t safety_flags; /* one-shot stop request and persistent stop latch */
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
	int64_t peak_height_um;
	int64_t max_height_um;
	int32_t terrain_peak_slope_cdeg;
	uint32_t last_timestamp_ms;
	uint32_t last_mag_timestamp_ms;
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
	uint8_t lift_overturn_count;
	uint8_t stop_latched;
	uint8_t safety_clear_count;
	uint8_t turn_event_time_valid;
	uint8_t heading_valid;
	uint8_t height_valid;
	uint8_t max_height_valid;
	uint8_t hump_state;
	uint8_t hump_armed;
	uint8_t hump_entry_count;
	uint8_t hump_flat_count;
	uint8_t hump_candidate_valid;
	int32_t peak_roll_cdeg;
	uint8_t motion_class;
} B4_PROCESSOR;

/* Caller serializes these with update. Rearm only for an accepted explicit START.
 * A recent, debounced safe acceleration observation is required. */
uint8_t b4_processor_restart_ready(const B4_PROCESSOR *processor, uint32_t now_ms);
uint8_t b4_processor_rearm(B4_PROCESSOR *processor, uint32_t now_ms);

/*
 * Fill a caller-owned config with documented tuning defaults. The axis
 * mapping follows +X left, +Y backward, +Z up.
 */
void b4_processor_default_config(B4_PROCESSOR_CONFIG *config);

/* Invalidate an in-progress hump on a sensor read error. Caller holds lock. */
void b4_processor_invalidate_hump(B4_PROCESSOR *processor);

/* Initialize processing state and the stationary startup calibrator. */
B4_CALIBRATION_STATUS b4_processor_init(
	B4_PROCESSOR *processor,
	const B4_PROCESSOR_CONFIG *config);

/*
 * Process one sample. Wheel odometry is retained for API compatibility;
 * the lateral roll-based hump height does not depend on wheel speed.
 */
B4_CALIBRATION_STATUS b4_processor_update(
	B4_PROCESSOR *processor,
	const B4_IMU_SAMPLE *sample,
	uint8_t speed_valid,
	int32_t speed_mm_s,
	B4_PROCESSOR_OUTPUT *output);

#endif /* B4_PROCESSOR_H */
