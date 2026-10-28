/*
 * b4_lsm303dlhc_decode.c
 * Hardware-independent LSM303DLHC output-register conversions.
 */

#include "b4_lsm303dlhc.h"

static int16_t b4_decode_i16_le(const uint8_t *bytes)
{
	uint16_t value = (uint16_t)bytes[0]
		       | (uint16_t)((uint16_t)bytes[1] << 8);

	return (int16_t)value;
}

static int16_t b4_decode_i16_be(const uint8_t *bytes)
{
	uint16_t value = (uint16_t)((uint16_t)bytes[0] << 8)
		       | (uint16_t)bytes[1];

	return (int16_t)value;
}

uint8_t b4_lsm303dlhc_decode_accel(const uint8_t bytes[6],
				   int16_t accel_mg[3])
{
	uint8_t axis;

	if ((bytes == 0) || (accel_mg == 0)) {
		return 0U;
	}

	for (axis = 0U; axis < 3U; axis++) {
		/* HR mode outputs signed 12-bit readings left-aligned in 16 bits. */
		accel_mg[axis] =
			(int16_t)(b4_decode_i16_le(&bytes[axis * 2U]) / 16);
	}

	return 1U;
}

uint8_t b4_lsm303dlhc_decode_mag(const uint8_t bytes[6],
				 int16_t mag_dut[3])
{
	int32_t raw_x;
	int32_t raw_z;
	int32_t raw_y;

	if ((bytes == 0) || (mag_dut == 0)) {
		return 0U;
	}

	/* At +/-1.3 gauss, sensitivity is 1100 LSB/Gauss for X/Y and 980 for Z.
	   1 Gauss is 100 microtesla, so the scale factor is 1000 deci-microtesla. */
	raw_x = b4_decode_i16_be(&bytes[0]);
	raw_z = b4_decode_i16_be(&bytes[2]);
	raw_y = b4_decode_i16_be(&bytes[4]);
	mag_dut[0] = (int16_t)((raw_x * 1000) / 1100);
	mag_dut[2] = (int16_t)((raw_z * 1000) / 980);
	mag_dut[1] = (int16_t)((raw_y * 1000) / 1100);

	return 1U;
}
