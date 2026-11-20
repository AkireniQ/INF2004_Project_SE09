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


/* Read all three sensors into one packed word; masked-off sensors count as white (0). */
EXPORT LF_SENSE line_sense(void)
{
    LF_SENSE s = {0, 0, 0, 1};
    INT raw, dk[3];
    INT i;

    for (i = 0; i < 3; i++) {
        dk[i] = 0;
        if (S3_IR_MASK & (1 << i)) {
            raw = ir_read(i);
            if (raw < 0) { s.ok = 0; return s; }    /* ADC error: skip this cycle */
            dk[i] = darkness(raw, white_raw[i], black_raw[i]);
        }
    }
    s.l = (UW)dk[0]; s.c = (UW)dk[1]; s.r = (UW)dk[2];
    return s;
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


    cmd->stop  = 0;
    cmd->left  = S3_CRUISE_RPM;
    cmd->right = S3_CRUISE_RPM;


    /* 1. Which way is the line pulling right now? */
    if (rd && !ld)      { dir =  1; }  /* right sensor on black: line is to the right */
    else if (ld && !rd) { dir = -1; }  /* left sensor on black: line is to the left   */


    if (dir != 0) {
        UW d = (dir > 0) ? DIR_RIGHT : DIR_LEFT;
        if (d == st->dir) {
            if (st->dark_ms < LF_MS_MAX - LINE_PERIOD_MS) { st->dark_ms += LINE_PERIOD_MS; }
        } else {
            st->dir     = d;           /* new side: start again from GENTLE */
            st->dark_ms = 0;
        }
        st->seek_ms = S3_SEEK_MS;      /* re-arm the seek for when it goes white */
        st->lost_ms = 0;
    } else if (ld && rd) {
        /* Both on black: junction or a wide marking. Drive straight over it. */
        st->dir = DIR_NONE; st->dark_ms = 0; st->seek_ms = 0; st->lost_ms = 0;
        return;
    } else if (st->seek_ms > 0 && st->dir != DIR_NONE) {
        /* SEEK: sensor went white while steering. Keep turning and keep
         * escalating; a short black pulse must not end the turn. */
        st->seek_ms = (st->seek_ms > LINE_PERIOD_MS) ? st->seek_ms - LINE_PERIOD_MS : 0;
        if (st->dark_ms < LF_MS_MAX - LINE_PERIOD_MS) { st->dark_ms += LINE_PERIOD_MS; }
        dir = (st->dir == DIR_RIGHT) ? 1 : -1;
    } else {
        /* Both white and no steer pending: straight. */
        st->dir     = DIR_NONE;
        st->dark_ms = 0;
        st->seek_ms = 0;
        if (cd || !(S3_IR_MASK & 0x2)) {
            st->lost_ms = 0;           /* centred (two-sensor mode cannot tell loss) */
        } else {
            /* Three sensors and none sees the line: coast, then stop. */
            if (st->lost_ms < LOST_TIMEOUT_MS) { st->lost_ms += LINE_PERIOD_MS; }
            if (st->lost_ms >= LOST_TIMEOUT_MS) {
                cmd->left  = LEVEL_STOPPED;
                cmd->right = LEVEL_STOPPED;
                cmd->stop  = 1;
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
    if (dir > 0) { cmd->left = (UH)outer; cmd->right = (UH)inner; }   /* turn right */
    else         { cmd->left = (UH)inner; cmd->right = (UH)outer; }   /* turn left  */
}


/* ---- junction actions ----------------------------------------------- */

/* Stop, drive the wheel centre onto the junction, then turn. Blocks until done. */
LOCAL BOOL do_turn(S3_ACTION a)
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


/* Carry out the mapped action.  FORWARD = keep following, nothing to do.
 * Turns hand the wheels to the direct motion API for the manoeuvre:
 * vehicle_invalidate_line() tells the controller we have no recommendation
 * while do_turn() blocks; the next vehicle_submit_line() resumes it. */
LOCAL BOOL junction_action(S3_ACTION act, LF_STATE *st)
{
    if (act == ACT_STRAIGHT) { return TRUE; }
    vehicle_invalidate_line();
    if (!do_turn(act)) { return FALSE; }
    *st = (LF_STATE){0};                        /* fresh steering memory on the new line */
    return TRUE;
}


/* One bit for the scanner: bar (1) or space (0), from DO or from AO+hysteresis. */
LOCAL INT centre_bar(INT *dark_out)
{
#ifdef S3_BC_DO_PIN
    BOOL v = ir_read_do(S3_BC_DO_PIN);
    INT bit = (S3_BC_DO_ACTIVE_LOW ? !v : v) ? 1 : 0;
    *dark_out = bit ? 1000 : 0;                 /* so the diag line shows C=1000 over a bar */
    return bit;
#else
    LOCAL UB in_bar = 0;                        /* 1 bit of hysteresis state */
    INT raw = ir_read(1);                       /* GP27 */
    INT d;
    if (raw < 0) { *dark_out = -1; return -1; }
    d = darkness(raw, white_raw[1], black_raw[1]);
    *dark_out = d;
    if (!in_bar && d > BC_TH_HIGH)    { in_bar = 1; }
    else if (in_bar && d < BC_TH_LOW) { in_bar = 0; }
    return in_bar;
#endif
}


LOCAL ID owner_id;
LOCAL void tick_release(void *exinf) { (void)exinf; (void)tk_wup_tsk(owner_id); }


/* Demo sequence per leg:
 *   1. drive, scanning Code 39 on the centre sensor (either direction);
 *      a decoded letter replaces 'stored'
 *   2. both outer sensors dark for JUNCTION_MIN_MS (and the scanner is not
 *      in the middle of a barcode) = cross junction
 *   3. look 'stored' up in the MQTT mapping -> FORWARD / U-TURN / LEFT / RIGHT
 *      no mapping yet or unknown letter -> stop and wait for the mapping
 *   4. perform it, clear the letter, blank junction detection briefly, repeat
 */
/* Task state, packed: 1+1+1+1+3+5+3+7+10+9 = 41 bits -> two 32-bit words. */
typedef struct {
    UW cbit          : 1;   /* last centre bit (for the S3_PRINT_CENTRE_BIT test print) */
    UW waiting       : 1;   /* stopped at a junction until the mapping resolves it */
    UW halted        : 1;   /* a manoeuvre failed: stay stopped                   */
    UW said_no_code  : 1;   /* "no barcode" message printed once                   */
    UW act           : 3;   /* S3_ACTION pending at the junction                   */
    UW stored        : 5;   /* decoded letter 1..26, 0 = none                      */
    UW tick          : 3;   /* 0..LINE_EVERY_N-1                                    */
    UW junc_ms       : 7;   /* 0..JUNCTION_MIN_MS                                   */
    UW junc_block_ms : 10;  /* 0..JUNC_BLANK_MS                                     */
    UW diag_ms       : 9;   /* 0..500                                               */
} LF_TASK;
#if (LINE_EVERY_N > 7) || (JUNCTION_MIN_MS > 127) || (JUNC_BLANK_MS > 1023)
#error "LF_TASK bit widths"
#endif
#define STORED_CHAR(ts) ((ts).stored ? (char)('A' + (ts).stored - 1) : (char)0)
#ifdef S3_DIRECT_MOTION
#define S3_FOLLOWING()  TRUE
#else
#define S3_FOLLOWING()  vehicle_is_following()
#endif


EXPORT void line_follow_task(INT stacd, void *exinf)
{
    LF_STATE  st = {0};
    LF_TASK   ts = {0};
    LF_CMD    cmd;
    LF_SENSE  s;
    T_CCYC    cyc = {0};
    INT  bar, d;
    char got, map[4];
    S3_ACTION act;
    UB   adc_err = 0;                           /* diagnostics: ADC read failures (saturating) */


    (void)stacd; (void)exinf;
    barcode_reset();
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


#ifdef S3_TEST_MAP
    if (s3_route_set_map(S3_TEST_MAP[0], S3_TEST_MAP[1], S3_TEST_MAP[2], S3_TEST_MAP[3]) == E_OK) {
        tm_printf((UB*)"[S3] TEST MAP active (no MQTT): F=%c U=%c L=%c R=%c\n",
                  S3_TEST_MAP[0], S3_TEST_MAP[1], S3_TEST_MAP[2], S3_TEST_MAP[3]);
    }
#endif
#ifdef S3_BC_DO_PIN
    tm_printf((UB*)"[S3] barcode source: DO on GP%d (active %s)\n", S3_BC_DO_PIN, S3_BC_DO_ACTIVE_LOW ? "LOW" : "HIGH");
#else
    tm_printf((UB*)"[S3] barcode source: AO (centre ADC, hysteresis %d/%d)\n", BC_TH_HIGH, BC_TH_LOW);
#endif

    while (1) {
        tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(owner_id);


        /* every tick: Code 39 scanner, one bit from the centre sensor */
        bar = centre_bar(&d);
#ifdef S3_PRINT_CENTRE_BIT                      /* TEST ONLY: print 1/0 when the centre bit changes */
        if (bar >= 0 && (UW)bar != ts.cbit) {
            ts.cbit = (UW)bar;
            tm_printf((UB*)"%d\n", bar);
        }
#endif
        if (bar >= 0) {
            barcode_feed(bar);
            got = barcode_take();
            if (got != 0) {
                ts.stored = (UW)(got - 'A' + 1);
                tm_printf((UB*)"[S3] barcode '%c' stored -> %s\n",
                          got, s3_action_name(s3_route_lookup(got)));
            }
        }


        if (++ts.tick < LINE_EVERY_N) { continue; }
        ts.tick = 0;


        /* every LINE_PERIOD_MS: line following */
        s = line_sense();
        if (!s.ok) {
            vehicle_invalidate_line();          /* controller has no recommendation this cycle */
            st = (LF_STATE){0};
            if (adc_err < 255) { adc_err++; }
            continue;
        }
        if (ts.junc_block_ms > 0) {
            ts.junc_block_ms = (ts.junc_block_ms > LINE_PERIOD_MS) ? ts.junc_block_ms - LINE_PERIOD_MS : 0;
        }


        /* ---- cross junction detection ----
         * Barcode bars also darken both outer sensors, so: no counting while
         * the scanner is busy, and JUNCTION_MIN_MS is longer than any bar. */
        if (!ts.halted && !ts.waiting && ts.junc_block_ms == 0 && S3_FOLLOWING() &&
            !barcode_busy() && s.l >= JUNC_DARK && s.r >= JUNC_DARK &&
            (st.dir == DIR_NONE || st.dark_ms < S3_HARD_MS)) {   /* not mid-pivot: a car turning hard
                                                                     crosses the line diagonally and
                                                                     darkens both sensors; that is not
                                                                     a junction */
            if (ts.junc_ms < JUNCTION_MIN_MS) { ts.junc_ms += LINE_PERIOD_MS; }
        } else {
            ts.junc_ms = 0;
        }


        act = (S3_ACTION)ts.act;
        if (ts.junc_ms >= JUNCTION_MIN_MS) {
            ts.junc_ms = 0;
            act = s3_route_lookup(STORED_CHAR(ts));
            s3_route_get(map);
            tm_printf((UB*)"[S3] JUNCTION: code='%c' map[F U L R]=%c%c%c%c action=%s\n",
                      ts.stored ? STORED_CHAR(ts) : '-', map[0], map[1], map[2], map[3], s3_action_name(act));
            if (act == ACT_NONE && S3_JUNCTION_DEFAULT != ACT_NONE) {
                act = S3_JUNCTION_DEFAULT;      /* unresolved junction: default action, keep moving */
                tm_printf((UB*)"[S3] no code/mapping for this junction: default %s\n", s3_action_name(act));
            }
            if (act == ACT_NONE) {
                ts.waiting = 1;                 /* stop here until the MQTT mapping makes sense of it */
                ts.said_no_code = 0;
                tm_printf((UB*)"[S3] waiting at junction for %s\n",
                          ts.stored ? "a mapping that includes this code" : "a barcode (none was read)");
            }
        }


        if (ts.waiting) {
            act = s3_route_lookup(STORED_CHAR(ts));
            if (act != ACT_NONE) {
                ts.waiting = 0;
                tm_printf((UB*)"[S3] mapping received: '%c' -> %s, resuming\n", STORED_CHAR(ts), s3_action_name(act));
            } else if (!ts.stored && !ts.said_no_code) {
                ts.said_no_code = 1;            /* nothing can resolve this leg; say so once */
                tm_printf((UB*)"[S3] no barcode for this junction: stopped\n");
            }
        }


        if (!ts.waiting && act != ACT_NONE) {   /* perform the junction action */
            if (!junction_action(act, &st)) {
                ts.halted = 1;
                tm_printf((UB*)"[S3] HALTED: %s failed\n", s3_action_name(act));
            }
            act = ACT_NONE;
            ts.stored = 0;                      /* the letter is consumed by this junction */
            barcode_reset();                    /* whatever the turn showed the centre sensor is not a barcode */
            ts.junc_block_ms = JUNC_BLANK_MS;
            ts.junc_ms = 0;
        }
        ts.act = (UW)act;


        line_decide((INT)s.l, (INT)s.c, (INT)s.r, &st, &cmd);
        if (barcode_busy() && st.dark_ms < S3_HARD_MS) {   /* steady, straight pass over the bars,
                                                               but never cancel a committed turn: on a
                                                               curve the centre sensor also sees edges */
            cmd.left = S3_CRUISE_RPM; cmd.right = S3_CRUISE_RPM; cmd.stop = 0;
        }
        if (ts.halted || ts.waiting) { cmd.left = 0; cmd.right = 0; cmd.stop = 1; }
#ifdef S3_DIRECT_MOTION                        /* TEST ONLY: bypass the vehicle controller */
        if (cmd.stop) { s3_motion_stop(); }
        else          { (void)s3_motion_set_wheel_rpm((INT)cmd.left, (INT)cmd.right); }
#else
        vehicle_submit_line((INT)s.l, (INT)s.c, (INT)s.r, (INT)cmd.left, (INT)cmd.right, (INT)cmd.stop, 0);
#endif


        ts.diag_ms += LINE_PERIOD_MS;
        if (ts.diag_ms >= 500) {
            ts.diag_ms = 0;
            tm_printf((UB*)"[S3] dark L=%d C=%d R=%d target RPM L=%d R=%d stop=%d dark_ms=%d seek=%d code='%c'%s%s%s busy=%d junc_ms=%d adc_err=%d follow=%d\n",
                      (INT)s.l, d, (INT)s.r, (INT)cmd.left, (INT)cmd.right, (INT)cmd.stop,
                      (INT)st.dark_ms, (INT)st.seek_ms,
                      ts.stored ? STORED_CHAR(ts) : '-', ts.waiting ? " WAITING" : "",
                      ts.halted ? " HALTED" : "", ts.junc_block_ms ? " BLANK" : "",
                      (INT)barcode_busy(), (INT)ts.junc_ms, (INT)adc_err, (INT)S3_FOLLOWING());
            barcode_diag();
            s3_motion_diagnostics();
        }
    }
}