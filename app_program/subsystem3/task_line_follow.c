/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel RPM out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"
#include "../vehicle_controller/vehicle_controller.h"


/* Calibration: raw ADC on white paper and on the black line (L, C, R).
   Measure on your track; the live check below prints the raw values. */
LOCAL const INT white_raw[3] = { 130, 130, 130 };
LOCAL const INT black_raw[3] = { 3200, 3200, 3200 };


LOCAL INT darkness(INT raw, INT white, INT black)
{
    INT d;
    if (black <= white) { return 0; }
    d = ((raw - white) * 1000) / (black - white);
    if (d < 0)    { d = 0; }
    if (d > 1000) { d = 1000; }
    return d;
}


/* Read all three sensors; masked-off sensors count as white (0). */
EXPORT ER line_sense(INT *l, INT *c, INT *r)
{
    INT raw[3];
    INT i;
    INT dk[3];


    for (i = 0; i < 3; i++) {
        if (S3_IR_MASK & (1 << i)) {
            raw[i] = ir_read(i);
            if (raw[i] < 0) { return E_IO; }     /* ADC error: skip this cycle */
            dk[i] = darkness(raw[i], white_raw[i], black_raw[i]);
        } else {
            dk[i] = 0;
        }
    }
    *l = dk[0]; *c = dk[1]; *r = dk[2];
    return E_OK;
}


/* PURE decision logic. l,c,r = darkness 0..1000. No hardware, no printing.
 *
 * Rules (two outer sensors ride on white, one each side of the line):
 *  - both white, nothing pending : straight at cruise.
 *  - one sensor on black         : steer toward it, GENTLE at once.
 *  - escalation by time since the steer started (dark_ms):
 *        >= S3_HARD_MS   HARD   (inner wheel stopped)
 *        >= S3_HARDER_MS HARDER (outer at S3_HARDER_RPM, inner stopped)
 *  - sensor white again (SEEK)   : the line is either back between the
 *        sensors or it slid past the sensor to the outside.  We cannot tell,
 *        so keep steering the same way, still escalating, until
 *          . the same sensor sees black again  -> keep going (line coming back)
 *          . the other sensor sees black        -> steer the other way, GENTLE
 *          . S3_SEEK_MS pass with both white    -> give up, straight
 *  - both on black (junction / wide bar): straight, the task handles it.
 * Three-sensor mode (S3_IR_MASK & 0x2): while the centre sensor sees the
 * line the steer stays GENTLE; all white for LOST_TIMEOUT_MS stops.
 * Call it once per LINE_PERIOD_MS; the timers live in LF_STATE.
 */
EXPORT void line_decide(INT l, INT c, INT r, LF_STATE *st, LF_CMD *cmd)
{
    BOOL ld = (l >= LINE_DARK);
    BOOL cd = (c >= LINE_DARK);
    BOOL rd = (r >= LINE_DARK);
    INT  dir = 0;                      /* <0 left, >0 right, 0 straight */
    INT  outer, inner;


    cmd->stop  = FALSE;
    cmd->left  = S3_CRUISE_RPM;
    cmd->right = S3_CRUISE_RPM;


    /* 1. Which way is the line pulling right now? */
    if (rd && !ld)      { dir =  1; }  /* right sensor on black: line is to the right */
    else if (ld && !rd) { dir = -1; }  /* left sensor on black: line is to the left   */


    if (dir != 0) {
        if (dir == st->last_pos) {
            if (st->dark_ms < 10000) { st->dark_ms += LINE_PERIOD_MS; }
        } else {
            st->last_pos = dir;        /* new side: start again from GENTLE */
            st->dark_ms  = 0;
        }
        st->seek_ms = S3_SEEK_MS;      /* re-arm the seek for when it goes white */
        st->lost_ms = 0;
    } else if (ld && rd) {
        /* Both on black: junction or a wide marking. Drive straight over it. */
        st->last_pos = 0; st->dark_ms = 0; st->seek_ms = 0; st->lost_ms = 0;
        return;
    } else if (st->seek_ms > 0 && st->last_pos != 0) {
        /* SEEK: sensor went white while steering. Keep turning and keep
         * escalating; a short black pulse must not end the turn. */
        st->seek_ms -= LINE_PERIOD_MS;
        if (st->dark_ms < 10000) { st->dark_ms += LINE_PERIOD_MS; }
        dir = st->last_pos;
    } else {
        /* Both white and no steer pending: straight. */
        st->last_pos = 0;
        st->dark_ms  = 0;
        st->seek_ms  = 0;
        if (cd || !(S3_IR_MASK & 0x2)) {
            st->lost_ms = 0;           /* centred (two-sensor mode cannot tell loss) */
        } else {
            /* Three sensors and none sees the line: coast, then stop. */
            if (st->lost_ms < LOST_TIMEOUT_MS) { st->lost_ms += LINE_PERIOD_MS; }
            if (st->lost_ms >= LOST_TIMEOUT_MS) {
                cmd->left  = LEVEL_STOPPED;
                cmd->right = LEVEL_STOPPED;
                cmd->stop  = TRUE;
            }
        }
        return;
    }


    /* 2. How hard? GENTLE first, harder the longer the steer has lasted.
     * RPM values stay even for the compact transport. */
    outer = S3_CURVE_RPM;
    inner = S3_INNER_RPM;
    if (!cd) {                         /* centre still on the line: stay gentle */
        if (st->dark_ms >= S3_HARD_MS)   { inner = LEVEL_STOPPED; }
        if (st->dark_ms >= S3_HARDER_MS) { outer = S3_HARDER_RPM; }
    }
    if (dir > 0) { cmd->left = outer; cmd->right = inner; }   /* turn right */
    else         { cmd->left = inner; cmd->right = outer; }   /* turn left  */
}


/* The original line_decide() above is unchanged. The controller owns motors;
 * the line task submits copied recommendations at the original cadence. */

typedef enum { ACT_NONE, ACT_STRAIGHT, ACT_LEFT, ACT_RIGHT, ACT_UTURN } ROUTE;


LOCAL ROUTE route_map(char code)
{
    switch (code) {
    case 'A': return ACT_STRAIGHT;
    case 'B': return ACT_UTURN;
    case 'C': return ACT_LEFT;
    case 'F': return ACT_RIGHT;
    default:  return ACT_NONE;        /* no barcode, or an unknown letter */
    }
}


LOCAL const char *act_name(ROUTE a)
{
    switch (a) {
    case ACT_STRAIGHT: return "STRAIGHT";
    case ACT_LEFT:     return "LEFT";
    case ACT_RIGHT:    return "RIGHT";
    case ACT_UTURN:    return "U-TURN";
    default:           return "NONE";
    }
}


/* Stop, drive the wheel centre onto the junction, then turn. Blocks until done. */
LOCAL BOOL do_turn(ROUTE a)
{
    s3_motion_stop();
    if (!s3_motion_move_mm(S3_ADVANCE_MM)) { return FALSE; }
    switch (a) {
    case ACT_LEFT:  return s3_motion_turn(-1, S3_TURN_90);
    case ACT_RIGHT: return s3_motion_turn( 1, S3_TURN_90);
    case ACT_UTURN: return s3_motion_turn(-1, 2 * S3_TURN_90);
    default:        return FALSE;
    }
}


LOCAL INT centre_dark(void)
{
    INT raw = ir_read(1);                       /* GP27 */
    return (raw < 0) ? -1 : darkness(raw, white_raw[1], black_raw[1]);
}


LOCAL ID owner_id;
LOCAL void tick_release(void *exinf) { (void)exinf; (void)tk_wup_tsk(owner_id); }


EXPORT void line_follow_task(INT stacd, void *exinf)
{
    LF_STATE st = { 0, 0, 0, 0 };
    LF_CMD   cmd;
    T_CCYC   cyc = {0};
    INT  l, c, r, d, tick = 0, diag_ms = 0;
    INT  junc_ms = 0, fl = 0, fr = 0;
    BOOL on_junction = FALSE, halted = FALSE;
    char stored = 0, got;
    ROUTE act;


    (void)stacd; (void)exinf;
    // barcode_reset(); //UNCOMMENT
    owner_id = tk_get_tid();
    cyc.cycatr = TA_HLNG | TA_STA;
    cyc.cychdr = (FP)tick_release;
    cyc.cyctim = S3_TICK_MS;
    cyc.cycphs = S3_TICK_MS;
    if (tk_cre_cyc(&cyc) < E_OK) {
        tm_printf((UB*)"[S3] tick cyclic creation failed\n");
        vehicle_invalidate_line();
        tk_ext_tsk();
        return;
    }


    while (1) {
        tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(owner_id);


        /* every tick: barcode on the centre sensor */ //UNCOMMENT
        // d = centre_dark();
        // if (d >= 0) {
        //     barcode_feed(d);
        //     static INT mx = 0, mn = 1000, t = 0;
        //     if (d > mx) { mx = d; } if (d < mn) { mn = d; }
        //     if (++t >= 500) { t = 0; tm_printf((UB*)"[S3] centre dark min=%d max=%d\n", mn, mx); mx = 0; mn = 1000; }
        //     got = barcode_take();
        //     if (got != 0) {
        //         stored = got;
        //         tm_printf((UB*)"[S3] barcode '%c' stored\n", got);
        //     }
        // }


        if (++tick < LINE_EVERY_N) { continue; }
        tick = 0;


        /* every LINE_EVERY_N-th tick (= LINE_PERIOD_MS): line following */
        if (line_sense(&l, &c, &r) != E_OK) {
            vehicle_invalidate_line();
            st = (LF_STATE){0};
            continue;
        }
        /* Use current readings: smoothing both sides delayed curve response. */


        //UNCOMMENT
        // if (barcode_busy()) { junc_ms = 0; }                 /* bars must not look like a junction */
        // else if (l >= JUNC_DARK && r >= JUNC_DARK) { junc_ms += LINE_PERIOD_MS; }
        // else { junc_ms = 0; on_junction = FALSE; }


        //COMMENT
        /* TEST: junction check without the barcode guard */
        if (!halted && vehicle_is_following() && l >= JUNC_DARK && r >= JUNC_DARK) { junc_ms += LINE_PERIOD_MS; }
        else { junc_ms = 0; on_junction = FALSE; }


        if (!on_junction && junc_ms >= JUNCTION_MIN_MS) {
            on_junction = TRUE;


            //COMMENT
            /* TEST: stop at the first cross junction and stay stopped */
            tm_printf((UB*)"[S3] JUNCTION (L=%d R=%d): STOP\n", l, r);
            halted = TRUE;


            //UNCOMMENT
            // act = route_map(stored);
            // tm_printf((UB*)"[S3] JUNCTION: code='%c' action=%s\n",
            //           stored ? stored : '-', act_name(act));
            // stored = 0;
            // barcode_reset();


            // if (act == ACT_NONE) {                           /* no usable code: stop and wait */
            //     s3_motion_stop();
            //     halted = TRUE;
            //     tm_printf((UB*)"[S3] HALTED at junction (no barcode)\n");
            //     continue;
            // }
            // if (act != ACT_STRAIGHT) {                       /* STRAIGHT just keeps following */
            //     if (!do_turn(act)) {
            //         halted = TRUE;
            //         tm_printf((UB*)"[S3] HALTED: turn failed\n");
            //         continue;
            //     }
            //     moving = FALSE; last_l = -1; last_r = -1; age_ms = 0;
            //     fl = 0; fr = 0; junc_ms = 0;
            //     st.last_pos = 0; st.dark_ms = 0; st.seek_ms = 0; st.lost_ms = 0;
            //     continue;                                    /* resume next cycle */
            // }
        }


        line_decide(l, c, r, &st, &cmd);
        if (halted) { cmd = (LF_CMD){0, 0, TRUE}; }
        vehicle_submit_line(l, c, r, cmd.left, cmd.right, cmd.stop, 0);
        diag_ms += LINE_PERIOD_MS;
        if (diag_ms >= 500) {
            diag_ms = 0;
            tm_printf((UB*)"[S3] dark L=%d C=%d R=%d target RPM L=%d R=%d stop=%d dark_ms=%d seek=%d\n",
                      l, c, r, cmd.left, cmd.right, cmd.stop, st.dark_ms, st.seek_ms);
            s3_motion_diagnostics();
        }
    }
}