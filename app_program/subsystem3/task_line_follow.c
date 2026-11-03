/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel levels out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"

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

/* PURE decision logic. l,c,r = darkness 0..1000. No hardware, no printing. */
#define S3_BASE_LEVEL 2      /* straight-line speed; 3 gives finer steps than 2 */
#define S3_DEADBAND 120    /* ignore |r - l| below this: drive straight */

EXPORT void line_decide(INT l, INT c, INT r, LF_STATE *st, LF_CMD *cmd) {
    INT err = r - l; /* + : line nearer the right sensor -> steer right */
    INT a = (err < 0) ? -err : err;
    INT cut;

    (void)c; (void)st;
    cmd->stop = FALSE;

    if (a < S3_DEADBAND) { cut = 0; }
    else if (a < 400) { cut = 1; }
    else if (a < 750) { cut = 2; }
    else { cut = 3; }

    cmd->left  = S3_BASE_LEVEL;
    cmd->right = S3_BASE_LEVEL;
    if (err > 0) { cmd->right = S3_BASE_LEVEL - cut; }   /* slow the inner (right) wheel */
    else if (err < 0) { cmd->left  = S3_BASE_LEVEL - cut; }
    if (cmd->left  < 0) { cmd->left  = 0; }
    if (cmd->right < 0) { cmd->right = 0; }
}

/* Send a command to Motion Control, only when it changes */
LOCAL void apply(const LF_CMD *cmd, BOOL *moving, INT *last_l, INT *last_r, INT *age_ms)
{
    *age_ms += LINE_PERIOD_MS;
    if (cmd->stop) {
        if (*moving) { s3_motion_stop(); *moving = FALSE; *last_l = -1; *last_r = -1; }
        return;
    }
    if (cmd->left != *last_l || cmd->right != *last_r || *age_ms >= S3_RESEND_MS) {
        if (s3_motion_set_wheel_levels(cmd->left, cmd->right)) {
            *moving = TRUE;
            *last_l = cmd->left;
            *last_r = cmd->right;
            *age_ms = 0;
        } else {
            *last_l = -1; *last_r = -1;   /* rejected: try again next cycle */
        }
    }
}

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
    LF_STATE st = { 0, 0 };
    LF_CMD   cmd;
    T_CCYC   cyc = {0};
    INT  l, c, r, d, tick = 0;
    INT  last_l = -1, last_r = -1, age_ms = 0, junc_ms = 0, fl = 0, fr = 0;
    BOOL moving = FALSE, on_junction = FALSE, halted = FALSE;
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

        if (halted) { continue; }
        if (++tick < LINE_EVERY_N) { continue; }
        tick = 0;

        /* every 10th tick: line following */
        if (line_sense(&l, &c, &r) != E_OK) { continue; }
        fl = (fl + l) / 2; fr = (fr + r) / 2;
        l = fl; r = fr;

        //UNCOMMENT
        // if (barcode_busy()) { junc_ms = 0; }                 /* bars must not look like a junction */
        // else if (l >= JUNC_DARK && r >= JUNC_DARK) { junc_ms += LINE_PERIOD_MS; }
        // else { junc_ms = 0; on_junction = FALSE; }

        //COMMENT
        /* TEST: junction check without the barcode guard */
        if (l >= JUNC_DARK && r >= JUNC_DARK) { junc_ms += LINE_PERIOD_MS; }
        else { junc_ms = 0; on_junction = FALSE; }

        if (!on_junction && junc_ms >= JUNCTION_MIN_MS) {
            on_junction = TRUE;

            //COMMENT
            /* TEST: stop at the first cross junction and stay stopped */
            tm_printf((UB*)"[S3] JUNCTION (L=%d R=%d): STOP\n", l, r);
            s3_motion_stop();
            halted = TRUE;
            continue;

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
            //     continue;                                    /* resume next cycle */
            // }
        }

        line_decide(l, c, r, &st, &cmd);
        apply(&cmd, &moving, &last_l, &last_r, &age_ms);
    }
}