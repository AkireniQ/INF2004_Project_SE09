/*
 * b4_event.c
 * Compact event validation and padding-independent serialization.
 */

#include "b4_event.h"

static void b4_event_write_u32_le(uint8_t *buffer, uint32_t value)
{
	buffer[0] = (uint8_t)value;
	buffer[1] = (uint8_t)(value >> 8);
	buffer[2] = (uint8_t)(value >> 16);
	buffer[3] = (uint8_t)(value >> 24);
}

static uint32_t b4_event_read_u32_le(const uint8_t *buffer)
{
	return ((uint32_t)buffer[0])
	     | ((uint32_t)buffer[1] << 8)
	     | ((uint32_t)buffer[2] << 16)
	     | ((uint32_t)buffer[3] << 24);
}

static uint32_t b4_event_encode_i32(int32_t value)
{
	if (value >= 0) {
		return (uint32_t)value;
	}

	return UINT32_MAX - (uint32_t)(-(value + 1));
}

static int32_t b4_event_decode_i32(uint32_t value)
{
	if (value <= (uint32_t)INT32_MAX) {
		return (int32_t)value;
	}

	return (int32_t)(-1 - (int32_t)(UINT32_MAX - value));
}

uint8_t b4_event_is_valid(const B4_EVENT *event)
{
	if (event == 0U) {
		return 0U;
	}

	if ((event->type < B4_EVENT_TILT) ||
	    (event->type > B4_EVENT_TERRAIN_END)) {
		return 0U;
	}

	if ((event->flags & (uint8_t)~B4_EVENT_FLAG_VALID_MASK) != 0U) {
		return 0U;
	}

	if (((event->flags & B4_EVENT_FLAG_VALUE_VALID) == 0U) &&
	    (event->value != 0)) {
		return 0U;
	}

	return 1U;
}

B4_EVENT_STATUS b4_event_encode(const B4_EVENT *event,
				uint8_t *buffer,
				uint32_t buffer_size)
{
	uint8_t encoded[B4_EVENT_ENCODED_SIZE];
	uint32_t value_bits;
	uint32_t index;

	if ((event == 0U) || (buffer == 0U)) {
		return B4_EVENT_STATUS_INVALID_ARGUMENT;
	}

	if (buffer_size < B4_EVENT_ENCODED_SIZE) {
		return B4_EVENT_STATUS_BUFFER_TOO_SMALL;
	}

	if (b4_event_is_valid(event) == 0U) {
		return B4_EVENT_STATUS_INVALID_EVENT;
	}

	b4_event_write_u32_le(encoded, event->timestamp_ms);
	encoded[4] = event->type;
	encoded[5] = event->flags;
	value_bits = b4_event_encode_i32(event->value);
	b4_event_write_u32_le(&encoded[6], value_bits);

	for (index = 0U; index < B4_EVENT_ENCODED_SIZE; index++) {
		buffer[index] = encoded[index];
	}

	return B4_EVENT_STATUS_OK;
}

B4_EVENT_STATUS b4_event_decode(const uint8_t *buffer,
				uint32_t buffer_size,
				B4_EVENT *event_out)
{
	B4_EVENT event;

	if ((buffer == 0U) || (event_out == 0U)) {
		return B4_EVENT_STATUS_INVALID_ARGUMENT;
	}

	if (buffer_size != B4_EVENT_ENCODED_SIZE) {
		return B4_EVENT_STATUS_INVALID_LENGTH;
	}

	event.timestamp_ms = b4_event_read_u32_le(buffer);
	event.type = buffer[4];
	event.flags = buffer[5];
	event.value = b4_event_decode_i32(b4_event_read_u32_le(&buffer[6]));

	if (b4_event_is_valid(&event) == 0U) {
		return B4_EVENT_STATUS_INVALID_EVENT;
	}

	*event_out = event;
	return B4_EVENT_STATUS_OK;
}
