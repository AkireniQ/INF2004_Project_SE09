/*
 * b4_lsm303dlhc.h
 * LSM303DLHC accelerometer/magnetometer driver for Buddy 4.
 */

#ifndef B4_LSM303DLHC_H
#define B4_LSM303DLHC_H

#include <stdint.h>

#include "b4_imu.h"

#define B4_LSM303DLHC_ACCEL_ADDRESS UINT8_C(0x19)
#define B4_LSM303DLHC_MAG_ADDRESS   UINT8_C(0x1e)

/*
 * Sensor handle. device_id is the micro T-Kernel "iica" device ID.
 * The handle is owned and accessed by one task only.
 */
typedef struct {
	int32_t device_id;
	uint8_t initialized;
	uint8_t accel_ctrl1;
	uint8_t accel_ctrl4;
	uint8_t accel_status;
	uint8_t accel_status_after_read;
	uint8_t accel_raw[6];
	uint8_t accel_raw_available;
	uint8_t accel_overrun_active;
	uint32_t accel_overrun_count;
	uint32_t poll_interval_ms;
	uint32_t poll_interval_max_ms;
	uint32_t poll_interval_us;
	uint32_t poll_interval_max_us;
	uint32_t last_poll_ms;
	uint32_t last_poll_us;
	uint8_t poll_time_valid;
	uint32_t accel_burst_us;
	uint32_t accel_status_after_us;
	uint32_t mag_status_us;
	uint32_t mag_data_us;
	uint32_t read_total_us;
} B4_LSM303DLHC;

typedef enum {
	B4_LSM303DLHC_STATUS_OK = 0,
	B4_LSM303DLHC_STATUS_INVALID_ARGUMENT,
	B4_LSM303DLHC_STATUS_NOT_INITIALIZED,
	B4_LSM303DLHC_STATUS_NO_FRESH_DATA,
	B4_LSM303DLHC_STATUS_ID_MISMATCH,
	B4_LSM303DLHC_STATUS_IO_ERROR
} B4_LSM303DLHC_STATUS;

/*
 * Open I2C0 device "iica", verify the LSM303DLHC IDs at addresses 0x19
 * and 0x1e, and configure acceleration for 100 Hz, +/-2 g high-resolution
 * mode and magnetometer for 75 Hz, +/-1.3 gauss, continuous conversion.
 * Returns an explicit error on open, identification, or configuration failure.
 */
B4_LSM303DLHC_STATUS b4_lsm303dlhc_init(B4_LSM303DLHC *sensor);

/*
 * Read fresh, uncalibrated measurements into sample_out. Acceleration is
 * represented in milli-g at 1 mg/LSB. Magnetometer data is in
 * deci-microtesla, using the configured +/-1.3 gauss gain and LSM303DLHC
 * axis sensitivities. Validity bits are set only for data-ready groups
 * without a lock indication. An accelerometer overrun means earlier
 * conversions were missed, not that the current BDU-protected XYZ reading
 * is incoherent: ACCEL_VALID remains set when ZYXDA is set, and
 * ACCEL_OVERRUN explicitly records the data loss. Returns NO_FRESH_DATA if
 * neither group has a fresh conversion.
 */
B4_LSM303DLHC_STATUS b4_lsm303dlhc_read(B4_LSM303DLHC *sensor,
					B4_IMU_SAMPLE *sample_out);

/*
 * Adapter for B4_IMU_SOURCE.read. Maps driver statuses to the model-independent
 * interface, including NO_FRESH_DATA to B4_IMU_STATUS_NO_DATA.
 */
B4_IMU_STATUS b4_lsm303dlhc_source_read(void *context,
					B4_IMU_SAMPLE *sample_out);

/*
 * Close the device. Safe to call for an uninitialized handle; reports close
 * errors instead of discarding them.
 */
B4_LSM303DLHC_STATUS b4_lsm303dlhc_close(B4_LSM303DLHC *sensor);

/*
 * Return the most recent raw T-Kernel error code from open/read/write/close,
 * or zero when no device API call has failed.
 */
int32_t b4_lsm303dlhc_last_error(void);

/*
 * Return the most recently observed accelerometer control/status registers
 * and raw output bytes. Raw bytes may be stale; accel_status must be checked
 * before interpreting them as a fresh conversion.
 */
uint8_t b4_lsm303dlhc_get_accel_diagnostics(
	const B4_LSM303DLHC *sensor,
	uint8_t *ctrl1,
	uint8_t *ctrl4,
	uint8_t *status,
	uint8_t *status_after_read,
	uint32_t *overrun_events,
	uint32_t *poll_interval_ms,
	uint32_t *poll_interval_max_ms,
	uint32_t *poll_interval_us,
	uint32_t *poll_interval_max_us,
	uint32_t *accel_burst_us,
	uint32_t *accel_status_after_us,
	uint32_t *mag_status_us,
	uint32_t *mag_data_us,
	uint32_t *read_total_us,
	uint8_t raw_xyz[6]);

/*
 * Decode the six little-endian, left-aligned accelerometer output bytes into
 * X/Y/Z milli-g, and the six big-endian magnetometer bytes in X/Z/Y register
 * order into canonical X/Y/Z deci-microtesla. These pure functions are also
 * used by host-side tests and do not access hardware.
 */
uint8_t b4_lsm303dlhc_decode_accel(const uint8_t bytes[6],
				   int16_t accel_mg[3]);
uint8_t b4_lsm303dlhc_decode_mag(const uint8_t bytes[6],
				 int16_t mag_dut[3]);

#endif /* B4_LSM303DLHC_H */
