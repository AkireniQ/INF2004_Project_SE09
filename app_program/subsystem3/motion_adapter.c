/*
 * motion_adapter.c: the ONLY file that talks to Buddy 2's Motion Control.
 * NOW (hand testing): prints the commands, only when they change.
 * LATER (sensors on the car): do the UNCOMMENT steps 1 to 3.
 */
#include "subsystem3.h"

/* UNCOMMENT (step 1 of 3) */
#include "../subsystem2/subsystem2.h" 

EXPORT void s3_motion_stop(void)
{
    tm_printf((UB*)"[MOTION] STOP\n");
    /* UNCOMMENT (step 2 of 3) */
    motion_stop();
}

EXPORT BOOL s3_motion_set_wheel_levels(INT left, INT right)
{
    LOCAL INT pl = -1, pr = -1;           /* last printed, avoids console spam from resends */

    if (left != pl || right != pr) {
        tm_printf((UB*)"[MOTION] wheels L=%d R=%d\n", left, right);
        pl = left; pr = right;
    }

    /* UNCOMMENT (step 3 of 3) : the whole block below */
    if (motion_faults() != 0) {
        tm_printf((UB*)"[MOTION] fault 0x%x, clearing\n", motion_faults());
        (void)motion_clear_faults();
        return FALSE;                     // retry next cycle after the clear is processed
    }
    if (!motion_set_wheel_levels((unsigned)left, (unsigned)right)) {
        return FALSE;                     // not ready, stop pending, or queue full
    }
    return TRUE;
}