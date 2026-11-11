/*
 * b4_safety_dispatch.h
 * Testable bridge from Buddy 4 safety flags to motor-stop callbacks.
 */

#ifndef B4_SAFETY_DISPATCH_H
#define B4_SAFETY_DISPATCH_H

#include <stdint.h>

#include "b4_processor.h"

typedef void (*B4_STOP_CALLBACK)(void);

typedef enum {
	B4_SAFETY_DISPATCH_NOT_REQUESTED = 0,
	B4_SAFETY_DISPATCH_DISPATCHED,
	B4_SAFETY_DISPATCH_INVALID_ARGUMENT
} B4_SAFETY_DISPATCH_STATUS;

B4_SAFETY_DISPATCH_STATUS b4_safety_dispatch_stop(
	uint8_t safety_flags,
	B4_STOP_CALLBACK motion_stop_callback,
	B4_STOP_CALLBACK vehicle_stop_callback);

#endif /* B4_SAFETY_DISPATCH_H */
