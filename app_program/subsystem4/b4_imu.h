/*
 * b4_imu.h
 * Model-independent IMU sample and source interface for Buddy 4.
 */

#ifndef B4_IMU_H
#define B4_IMU_H

#include <stdint.h>

#define B4_IMU_AXIS_COUNT 3U

/* Sample validity bits. All other bits are reserved and must be zero. */
#define B4_IMU_SAMPLE_FLAG_ACCEL_VALID UINT8_C(0x01)
#define B4_IMU_SAMPLE_FLAG_MAG_VALID   UINT8_C(0x02)
#define B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN UINT8_C(0x04)
#define B4_IMU_SAMPLE_FLAG_VALID_MASK \
	(B4_IMU_SAMPLE_FLAG_ACCEL_VALID | B4_IMU_SAMPLE_FLAG_MAG_VALID | \
	 B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN)

/*
 * Model-independent sample using canonical units:
 * - timestamp_ms: milliseconds since the producer's chosen epoch; wraps
 *   modulo 2^32.
 * - accel_mg: acceleration in milli-g (1 g = 1000 mg), ordered X, Y, Z.
 *   Each component is limited by int16_t to -32768..32767 mg.
 * - mag_dut: magnetic flux density in deci-microtesla
 *   (0.1 microtesla per unit), ordered X, Y, Z. This signed 16-bit scale
 *   covers -3276.8 to 3276.7 microtesla.
 * - flags: measurement validity and diagnostics. ACCEL_OVERRUN records
 *   missed accelerometer conversions; it does not invalidate the current
 *   coherent acceleration value when ACCEL_VALID is also set.
 *
 * A producer must not silently clamp measurements that cannot be represented.
 */
typedef struct {
	uint32_t timestamp_ms;
	int16_t mag_dut[B4_IMU_AXIS_COUNT];
	int16_t accel_mg[B4_IMU_AXIS_COUNT];
	uint8_t flags;
} B4_IMU_SAMPLE;

typedef enum {
	B4_IMU_STATUS_OK = 0,
	B4_IMU_STATUS_INVALID_ARGUMENT,
	B4_IMU_STATUS_INVALID_SAMPLE,
	B4_IMU_STATUS_NO_DATA,
	B4_IMU_STATUS_IO_ERROR
} B4_IMU_STATUS;

/*
 * A source reads one sample into sample_out. Implementations may represent
 * real hardware or synthetic/test data; context is owned by the caller.
 */
typedef B4_IMU_STATUS (*B4_IMU_READ_FN)(void *context,
					B4_IMU_SAMPLE *sample_out);

typedef struct {
	B4_IMU_READ_FN read;
	void *context;
} B4_IMU_SOURCE;

/*
 * Return nonzero when the sample has at least one valid measurement group,
 * has no reserved flags, and does not report overrun without valid
 * acceleration. A null pointer is invalid.
 */
uint8_t b4_imu_sample_is_valid(const B4_IMU_SAMPLE *sample);

/*
 * Read and validate one sample from source. sample_out is unchanged on
 * failure. Returns the source's status, or an argument/sample error.
 */
B4_IMU_STATUS b4_imu_read(const B4_IMU_SOURCE *source,
			  B4_IMU_SAMPLE *sample_out);

#endif /* B4_IMU_H */
