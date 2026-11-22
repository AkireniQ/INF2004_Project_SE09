/*
 * b4_safety_dispatch.c
 * Small, host-testable dispatch for the Buddy 4 safety stop request.
 */

#include "b4_safety_dispatch.h"

B4_SAFETY_DISPATCH_STATUS b4_safety_dispatch_stop(
	uint8_t safety_flags,
	B4_STOP_CALLBACK motion_stop_callback,
	B4_STOP_CALLBACK vehicle_stop_callback)
{
	if ((safety_flags & B4_PROCESSOR_FLAG_STOP_REQUESTED) == 0U) {
		return B4_SAFETY_DISPATCH_NOT_REQUESTED;
	}
	if ((motion_stop_callback == 0) || (vehicle_stop_callback == 0)) {
		return B4_SAFETY_DISPATCH_INVALID_ARGUMENT;
	}

	motion_stop_callback();
	vehicle_stop_callback();
	return B4_SAFETY_DISPATCH_DISPATCHED;
}
