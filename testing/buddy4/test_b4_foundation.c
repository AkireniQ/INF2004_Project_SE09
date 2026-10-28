/*
 * Host-side tests for Buddy 4 model-independent data and event modules.
 * Build from the repository root with:
 *   clang -std=c11 -Wall -Wextra -Werror -pedantic \
 *     app_program/subsystem4/b4_imu.c \
 *     app_program/subsystem4/b4_lsm303dlhc_decode.c \
 *     app_program/subsystem4/b4_event.c \
 *     testing/buddy4/test_b4_foundation.c -o /tmp/test_b4_foundation
 */

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/subsystem4/b4_event.h"
#include "../../app_program/subsystem4/b4_imu.h"
#include "../../app_program/subsystem4/b4_lsm303dlhc.h"

static int test_failures;

#define CHECK(condition) \
	do { \
		if (!(condition)) { \
			(void)printf("FAIL %s:%d: %s\n", \
				     __FILE__, __LINE__, #condition); \
			test_failures++; \
		} \
	} while (0)

typedef struct {
	B4_IMU_STATUS status;
	B4_IMU_SAMPLE sample;
} SYNTHETIC_SOURCE;

static B4_IMU_STATUS synthetic_read(void *context,
				    B4_IMU_SAMPLE *sample_out)
{
	const SYNTHETIC_SOURCE *source = context;

	if (source == 0) {
		return B4_IMU_STATUS_INVALID_ARGUMENT;
	}

	if (source->status == B4_IMU_STATUS_OK) {
		*sample_out = source->sample;
	}

	return source->status;
}

static void test_sample_representation(void)
{
	B4_IMU_SAMPLE sample = {0};
	B4_IMU_SAMPLE result = {0};
	SYNTHETIC_SOURCE context = {0};
	B4_IMU_SOURCE source;

	CHECK(sizeof(sample.timestamp_ms) == sizeof(uint32_t));
	CHECK(sizeof(sample.accel_mg[0]) == sizeof(int16_t));
	CHECK(sizeof(sample.mag_dut[0]) == sizeof(int16_t));
	CHECK(sizeof(sample) <= 20U);
	CHECK(b4_imu_sample_is_valid(0) == 0U);
	CHECK(b4_imu_sample_is_valid(&sample) == 0U);

	sample.timestamp_ms = UINT32_MAX;
	sample.accel_mg[0] = INT16_MIN;
	sample.accel_mg[1] = INT16_MAX;
	sample.mag_dut[2] = INT16_MIN;
	sample.flags = B4_IMU_SAMPLE_FLAG_ACCEL_VALID;
	CHECK(b4_imu_sample_is_valid(&sample) != 0U);

	sample.flags = B4_IMU_SAMPLE_FLAG_MAG_VALID;
	CHECK(b4_imu_sample_is_valid(&sample) != 0U);

	sample.flags = B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN;
	CHECK(b4_imu_sample_is_valid(&sample) == 0U);
	sample.flags = B4_IMU_SAMPLE_FLAG_ACCEL_VALID
		     | B4_IMU_SAMPLE_FLAG_ACCEL_OVERRUN;
	CHECK(b4_imu_sample_is_valid(&sample) != 0U);

	sample.flags = UINT8_C(0x80);
	CHECK(b4_imu_sample_is_valid(&sample) == 0U);

	context.status = B4_IMU_STATUS_OK;
	context.sample = (B4_IMU_SAMPLE){0};
	context.sample.timestamp_ms = 1234U;
	context.sample.accel_mg[0] = -1000;
	context.sample.mag_dut[2] = 750;
	context.sample.flags = B4_IMU_SAMPLE_FLAG_ACCEL_VALID
			     | B4_IMU_SAMPLE_FLAG_MAG_VALID;
	source.read = synthetic_read;
	source.context = &context;

	CHECK(b4_imu_read(&source, &result) == B4_IMU_STATUS_OK);
	CHECK(result.timestamp_ms == 1234U);
	CHECK(result.accel_mg[0] == -1000);
	CHECK(result.mag_dut[2] == 750);

	result.timestamp_ms = 77U;
	context.status = B4_IMU_STATUS_NO_DATA;
	CHECK(b4_imu_read(&source, &result) == B4_IMU_STATUS_NO_DATA);
	CHECK(result.timestamp_ms == 77U);

	context.status = B4_IMU_STATUS_OK;
	context.sample.flags = 0U;
	CHECK(b4_imu_read(&source, &result) == B4_IMU_STATUS_INVALID_SAMPLE);
	CHECK(result.timestamp_ms == 77U);
	CHECK(b4_imu_read(0, &result) == B4_IMU_STATUS_INVALID_ARGUMENT);
	CHECK(b4_imu_read(&source, 0) == B4_IMU_STATUS_INVALID_ARGUMENT);
}

static void test_event_representation_and_encoding(void)
{
	B4_EVENT event = {0};
	B4_EVENT decoded = {0};
	uint8_t encoded[B4_EVENT_ENCODED_SIZE];
	static const uint8_t expected[B4_EVENT_ENCODED_SIZE] = {
		0x78U, 0x56U, 0x34U, 0x12U,
		B4_EVENT_HUMP_PEAK_HEIGHT,
		B4_EVENT_FLAG_VALUE_VALID,
		0x00U, 0x00U, 0x00U, 0x80U
	};

	CHECK(sizeof(event.timestamp_ms) == sizeof(uint32_t));
	CHECK(sizeof(event.value) == sizeof(int32_t));
	CHECK(sizeof(event.type) == sizeof(uint8_t));
	CHECK(sizeof(event.flags) == sizeof(uint8_t));
	CHECK(sizeof(event) <= 12U);
	CHECK(B4_EVENT_ENCODED_SIZE == 10U);
	CHECK(b4_event_is_valid(0) == 0U);

	event.timestamp_ms = UINT32_C(0x12345678);
	event.type = B4_EVENT_HUMP_PEAK_HEIGHT;
	event.flags = B4_EVENT_FLAG_VALUE_VALID;
	event.value = INT32_MIN;
	CHECK(b4_event_is_valid(&event) != 0U);

	(void)memset(encoded, 0xA5, sizeof(encoded));
	CHECK(b4_event_encode(&event, encoded, sizeof(encoded))
	      == B4_EVENT_STATUS_OK);
	CHECK(memcmp(encoded, expected, sizeof(expected)) == 0);

	CHECK(b4_event_decode(encoded, sizeof(encoded), &decoded)
	      == B4_EVENT_STATUS_OK);
	CHECK(decoded.timestamp_ms == event.timestamp_ms);
	CHECK(decoded.type == event.type);
	CHECK(decoded.flags == event.flags);
	CHECK(decoded.value == event.value);

	event.value = INT32_MAX;
	CHECK(b4_event_encode(&event, encoded, sizeof(encoded))
	      == B4_EVENT_STATUS_OK);
	CHECK(b4_event_decode(encoded, sizeof(encoded), &decoded)
	      == B4_EVENT_STATUS_OK);
	CHECK(decoded.value == INT32_MAX);

	event.flags = 0U;
	event.value = 0;
	CHECK(b4_event_is_valid(&event) != 0U);
	event.value = 1;
	CHECK(b4_event_is_valid(&event) == 0U);
	event.value = 0;
	event.flags = UINT8_C(0x80);
	CHECK(b4_event_is_valid(&event) == 0U);
	event.flags = 0U;
	event.type = UINT8_C(0xFF);
	CHECK(b4_event_is_valid(&event) == 0U);

	event.type = B4_EVENT_COLLISION;
	event.flags = B4_EVENT_FLAG_VALUE_VALID;
	event.value = -2500;
	CHECK(b4_event_encode(&event, encoded, sizeof(encoded) - 1U)
	      == B4_EVENT_STATUS_BUFFER_TOO_SMALL);
	CHECK(b4_event_decode(encoded, sizeof(encoded) - 1U, &decoded)
	      == B4_EVENT_STATUS_INVALID_LENGTH);

	event.flags = 0U;
	event.value = 3;
	CHECK(b4_event_encode(&event, encoded, sizeof(encoded))
	      == B4_EVENT_STATUS_INVALID_EVENT);

	event.flags = B4_EVENT_FLAG_VALUE_VALID;
	event.value = -2500;
	CHECK(b4_event_encode(&event, encoded, sizeof(encoded))
	      == B4_EVENT_STATUS_OK);
	encoded[4] = UINT8_C(0xFF);
	CHECK(b4_event_decode(encoded, sizeof(encoded), &decoded)
	      == B4_EVENT_STATUS_INVALID_EVENT);

	event.type = B4_EVENT_TERRAIN_START;
	event.value = 1200;
	CHECK(b4_event_is_valid(&event) != 0U);
	event.type = B4_EVENT_CALIBRATION_READY;
	event.flags = 0U;
	event.value = 0;
	CHECK(b4_event_is_valid(&event) != 0U);
	event.type = B4_EVENT_TERRAIN_END;
	CHECK(b4_event_is_valid(&event) != 0U);
}

static void test_lsm303dlhc_register_conversion(void)
{
	static const uint8_t accel_bytes[6] = {
		0x80U, 0x3eU, 0xc0U, 0xe0U, 0x00U, 0x80U
	};
	static const uint8_t mag_bytes[6] = {
		0x02U, 0x26U, 0xfeU, 0x16U, 0xfbU, 0xb4U
	};
	int16_t accel_mg[3] = {0, 0, 0};
	int16_t mag_dut[3] = {0, 0, 0};

	CHECK(b4_lsm303dlhc_decode_accel(accel_bytes, accel_mg) != 0U);
	CHECK(accel_mg[0] == 1000);
	CHECK(accel_mg[1] == -500);
	CHECK(accel_mg[2] == -2048);
	CHECK(b4_lsm303dlhc_decode_accel(0, accel_mg) == 0U);
	CHECK(b4_lsm303dlhc_decode_accel(accel_bytes, 0) == 0U);

	CHECK(b4_lsm303dlhc_decode_mag(mag_bytes, mag_dut) != 0U);
	CHECK(mag_dut[0] == 500);
	CHECK(mag_dut[1] == -1000);
	CHECK(mag_dut[2] == -500);
	CHECK(b4_lsm303dlhc_decode_mag(0, mag_dut) == 0U);
	CHECK(b4_lsm303dlhc_decode_mag(mag_bytes, 0) == 0U);
}

int main(void)
{
	test_sample_representation();
	test_event_representation_and_encoding();
	test_lsm303dlhc_register_conversion();

	if (test_failures != 0) {
		(void)printf("%d test check(s) failed\n", test_failures);
		return 1;
	}

	(void)printf("Buddy 4 foundation tests passed\n");
	return 0;
}
