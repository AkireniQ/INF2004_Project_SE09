/*
 * task_line_follow.c: line following (Buddy 3).
 * line_sense()  reads the 3 sensors -> darkness 0..1000
 * line_decide() PURE logic: darkness in, wheel levels out (easy to unit-test)
 * line_follow_task() runs both every LINE_PERIOD_MS and drives Motion Control
 */
#include "subsystem3.h"

/* Calibration: raw ADC on white paper and on the black line (L, C, R).
   Measure on your track; the live check below prints the raw values. */
LOCAL const INT white_raw[3] = {  500,  500,  500 };
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
EXPORT void line_decide(INT l, INT c, INT r, LF_STATE *st, LF_CMD *cmd)
{
    INT sum = l + c + r;
    INT pos;

    cmd->stop = FALSE;

    if (sum >= LOST_SUM_MAX) {
        /* Line visible. pos: -1000 = far left, 0 = centred, +1000 = far right */
        pos = ((r - l) * 1000) / sum;
        st->last_pos = pos;
        st->lost_ms  = 0;

        if (pos > -POS_STRAIGHT && pos < POS_STRAIGHT) {
            cmd->left = LEVEL_CRUISE; cmd->right = LEVEL_CRUISE;   /* straight */
        } else if (pos >= POS_STRAIGHT && pos < POS_GENTLE) {
            cmd->left = LEVEL_CRUISE; cmd->right = LEVEL_SLOW;     /* gentle right */
        } else if (pos <= -POS_STRAIGHT && pos > -POS_GENTLE) {
            cmd->left = LEVEL_SLOW;   cmd->right = LEVEL_CRUISE;   /* gentle left */
        } else if (pos >= POS_GENTLE) {
            cmd->left = LEVEL_CRUISE; cmd->right = LEVEL_STOPPED;  /* sharp right */
        } else {
            cmd->left = LEVEL_STOPPED; cmd->right = LEVEL_CRUISE;  /* sharp left */
        }
    } else {
        /* Line lost: pivot toward where it was last seen, then give up */
        st->lost_ms += LINE_PERIOD_MS;
        if (st->lost_ms < LOST_TIMEOUT_MS) {
            if (st->last_pos >= 0) {
                cmd->left = LEVEL_CRUISE;  cmd->right = LEVEL_STOPPED;
            } else {
                cmd->left = LEVEL_STOPPED; cmd->right = LEVEL_CRUISE;
            }
        } else {
            cmd->left = LEVEL_STOPPED; cmd->right = LEVEL_STOPPED;
            cmd->stop = TRUE;
        }
    }
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
EXPORT void line_follow_task(INT stacd, void *exinf)
{
    LF_STATE st = { 0, 0 };
    LF_CMD cmd;
    INT l, c, r;
    INT last_l = -1, last_r = -1;
    INT junc_ms = 0; 
    INT age_ms = 0;
    BOOL on_junction = FALSE;
    BOOL moving = FALSE;

    (void)stacd; (void)exinf;

    while (1) {
        if (line_sense(&l, &c, &r) == E_OK) {
            if (l >= JUNC_DARK && r >= JUNC_DARK) {
                junc_ms += LINE_PERIOD_MS;
            } else {
                junc_ms = 0;
                on_junction = FALSE; /* re-arm once we leave the junction */
            }
            if (!on_junction && junc_ms >= JUNCTION_MIN_MS) {
                on_junction = TRUE;
                tm_printf((UB*)"[S3] JUNCTION detected (L=%d R=%d)\n", l, r);
                /* HOOK: call the route action here (look up the stored barcode letter,
                then forward / U-turn / left / right). */
            }
            line_decide(l, c, r, &st, &cmd);
            apply(&cmd, &moving, &last_l, &last_r, &age_ms);
        }
        tk_dly_tsk(LINE_PERIOD_MS);
    }
}