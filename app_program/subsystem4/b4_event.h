/*
 * b4_event.h
 * Compact Buddy 4 motion and terrain event representation.
 */

#ifndef B4_EVENT_H
#define B4_EVENT_H

#include <stdint.h>

#define B4_EVENT_ENCODED_SIZE 10U

/* Event value is present; remaining flag bits are reserved and must be zero. */
#define B4_EVENT_FLAG_VALUE_VALID UINT8_C(0x01)
#define B4_EVENT_FLAG_VALID_MASK  B4_EVENT_FLAG_VALUE_VALID

/* Stable one-byte event codes. These define representation, not thresholds. */
#define B4_EVENT_TILT                 UINT8_C(0x01)
#define B4_EVENT_RAMP_DETECTED        UINT8_C(0x02)
#define B4_EVENT_HUMP_DETECTED        UINT8_C(0x03)
#define B4_EVENT_HUMP_PEAK_HEIGHT     UINT8_C(0x04)
#define B4_EVENT_MOTION_CLASS         UINT8_C(0x05)
#define B4_EVENT_COLLISION            UINT8_C(0x06)
#define B4_EVENT_TURN_RATE             UINT8_C(0x07)
#define B4_EVENT_CALIBRATION_READY     UINT8_C(0x08)
#define B4_EVENT_TERRAIN_START         UINT8_C(0x09)
#define B4_EVENT_TERRAIN_END           UINT8_C(0x0a)

/*
 * Fixed-size event record:
 * - timestamp_ms: milliseconds since the producer's chosen epoch; wraps
 *   modulo 2^32.
 * - type: one of the B4_EVENT_* codes.
 * - flags: B4_EVENT_FLAG_VALUE_VALID when value contains a measurement or
 *   code; otherwise value must be zero.
 * - value: signed 32-bit type-specific value. Units are centidegrees for
 *   tilt, terrain-start slope, ramp slope, and hump slope; millimetres for
 *   an odometry-assisted peak-height estimate; milli-g for collision peak
 *   dynamic acceleration; and milli-degrees/second for estimated magnetic
 *   heading rate. Motion-class values use the B4_MOTION_CLASS codes. A
 *   terrain-end and calibration-ready event carry no value.
 *   Detection defaults are tuning values, not project-specified thresholds.
 */
typedef struct {
	uint32_t timestamp_ms;
	int32_t value;
	uint8_t type;
	uint8_t flags;
} B4_EVENT;

typedef enum {
	B4_EVENT_STATUS_OK = 0,
	B4_EVENT_STATUS_INVALID_ARGUMENT,
	B4_EVENT_STATUS_INVALID_EVENT,
	B4_EVENT_STATUS_BUFFER_TOO_SMALL,
	B4_EVENT_STATUS_INVALID_LENGTH
} B4_EVENT_STATUS;

/*
 * Validate event type, flags, and canonical no-value representation.
 * Returns nonzero when valid; null is invalid.
 */
uint8_t b4_event_is_valid(const B4_EVENT *event);

/*
 * Encode one event as exactly B4_EVENT_ENCODED_SIZE bytes:
 * timestamp (uint32 little-endian), type, flags, value (int32 little-endian).
 * No C structure padding is included.
 */
B4_EVENT_STATUS b4_event_encode(const B4_EVENT *event,
				uint8_t *buffer,
				uint32_t buffer_size);

/*
 * Decode exactly B4_EVENT_ENCODED_SIZE bytes. event_out is unchanged on
 * failure; malformed types, flags, or non-canonical values are rejected.
 */
B4_EVENT_STATUS b4_event_decode(const uint8_t *buffer,
				uint32_t buffer_size,
				B4_EVENT *event_out);

#endif /* B4_EVENT_H */
