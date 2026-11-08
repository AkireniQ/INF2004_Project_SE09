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

EXPORT BOOL s3_motion_set_wheel_rpm(INT left, INT right)
{
    LOCAL unsigned reported_fault = 0;
    LOCAL INT pl = -1, pr = -1;           /* last printed, avoids console spam from resends */

    if (left != pl || right != pr) {
        tm_printf((UB*)"[MOTION] target RPM L=%d R=%d\n", left, right);
        pl = left; pr = right;
    }

    /* UNCOMMENT (step 3 of 3) : the whole block below */
    unsigned fault = motion_faults();
    if (fault != 0) {
        if (fault != reported_fault) {
            tm_printf((UB*)"[MOTION] latched fault 0x%x: inspect cause before explicit reset\n", fault);
        }
        reported_fault = fault;
        return FALSE;
    }
    reported_fault = 0;
    if (!motion_set_wheel_rpm((unsigned)left, (unsigned)right)) {
        return FALSE;                     // not ready, stop pending, or queue full
    }
    return TRUE;
}

/* Submit a bounded move/turn, wait until Buddy 2 accepts it, then until it finishes. */
LOCAL BOOL run_blocking(bool (*fn)(unsigned), unsigned arg, INT max_ms)
{
    S2_REPLY rep;
    uint32_t seq0;
    INT tries, t;

    for (tries = 0; tries < 10; tries++) {
        (void)motion_get_reply(&rep);
        seq0 = rep.sequence;
        if (!fn(arg)) { tk_dly_tsk(50); continue; }          /* stop pending / queue full */
        for (t = 0; t < 100; t++) {                          /* wait for the owner (up to ~1 s) */
            tk_dly_tsk(10);
            (void)motion_get_reply(&rep);
            if (rep.sequence != seq0) { break; }
        }
        if (rep.sequence == seq0) { continue; }
        if (!rep.accepted) { tk_dly_tsk(50); continue; }     /* e.g. still settling: retry */
        for (t = 0; t < max_ms / 10; t++) {                  /* accepted: wait for completion */
            tk_dly_tsk(10);
            if (!motion_busy()) { return TRUE; }
        }
        return FALSE;
    }
    return FALSE;
}

EXPORT BOOL s3_motion_move_mm(INT mm)
{
    tm_printf((UB*)"[MOTION] move forward %d mm\n", mm);
    return run_blocking(motion_move_forward_mm, (unsigned)mm, 5000);
}

EXPORT BOOL s3_motion_turn(INT dir, INT steps)
{
    tm_printf((UB*)"[MOTION] turn %s %d steps (%d deg)\n",
              dir < 0 ? "LEFT" : "RIGHT", steps, steps * 45);
    return run_blocking(dir < 0 ? motion_turn_left : motion_turn_right, (unsigned)steps, 8000);
}
/* Called by the line task, never from the time-critical motor owner. */
EXPORT void s3_motion_diagnostics(void)
{
    S2_STATUS status;
    if (!motion_get_status(&status)) { return; }
    tm_printf((UB*)"[MOTION] actual rpm L=%u R=%u PWM L=%u%% R=%u%% faults=0x%x\n",
              s2_get_bits(&status, 9, 9), s2_get_bits(&status, 18, 9),
              s2_get_bits(&status, 27, 7), s2_get_bits(&status, 34, 7),
              s2_get_bits(&status, 5, 4));
}
