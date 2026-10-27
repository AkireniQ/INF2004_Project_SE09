/*
 * b4_calibration.h
 * Stationary accelerometer calibration for Buddy 4.
 */

#ifndef B4_CALIBRATION_H
#define B4_CALIBRATION_H

#include <stdint.h>

#include "b4_imu.h"

#define B4_CALIBRATION_DEFAULT_SAMPLES       UINT16_C(100)
#define B4_CALIBRATION_DEFAULT_MIN_NORM_MG   UINT16_C(850)
#define B4_CALIBRATION_DEFAULT_MAX_NORM_MG   UINT16_C(1150)
#define B4_CALIBRATION_DEFAULT_MAX_VAR_MG2   UINT32_C(2500)

typedef enum {
	B4_CALIBRATION_COLLECTING = 0,
	B4_CALIBRATION_COMPLETE,
	B4_CALIBRATION_MOVING,
	B4_CALIBRATION_BAD_GRAVITY,
	B4_CALIBRATION_INVALID_ARGUMENT
} B4_CALIBRATION_STATUS;

/*
 * Configuration values are engineering defaults, not project-prescribed
 * limits. Variance is per axis in (mg)^2; all settings are caller-adjustable.
 */
typedef struct {
	uint16_t required_samples;
	uint16_t min_norm_mg;
	uint16_t max_norm_mg;
	uint32_t max_axis_variance_mg2;
} B4_CALIBRATION_CONFIG;

typedef struct {
	B4_CALIBRATION_CONFIG config;
	int64_t sum_mg[3];
	uint64_t sum_squares_mg2[3];
	int16_t baseline_mg[3];
	uint16_t sample_count;
	uint8_t ready;
} B4_CALIBRATOR;

/*
 * Initialize a calibrator. A null config selects documented engineering
 * defaults; invalid zero/range settings are rejected.
 */
B4_CALIBRATION_STATUS b4_calibration_init(
	B4_CALIBRATOR *calibrator,
	const B4_CALIBRATION_CONFIG *config);

/*
 * Add one fresh acceleration sample. Missing/stale samples do not advance
 * calibration. Motion resets the current collection so the baseline cannot
 * be silently accepted from a moving vehicle.
 */
B4_CALIBRATION_STATUS b4_calibration_add(
	B4_CALIBRATOR *calibrator,
	const B4_IMU_SAMPLE *sample);

/*
 * Return the calibrated mean gravity vector in mg; only succeeds after
 * successful stationary calibration.
 */
uint8_t b4_calibration_get_baseline(
	const B4_CALIBRATOR *calibrator,
	int16_t baseline_mg[3]);

#endif /* B4_CALIBRATION_H */
