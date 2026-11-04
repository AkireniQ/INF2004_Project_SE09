/*
 *  subsystem3.h
 *  Subsystem 3 -- Consumer (statistics).  Student ID: <student ID>.
 *
 *  Public interface of subsystem 3.  Other subsystems include it as
 *  "../subsystem3/subsystem3.h".
 */


#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H


#include "../app.h"


#define S3_PRI_CONSUMER     16  /* priority band 16-17 (TEAM.md) */
#define S3_BATCH_SIZE       5   /* report to subsystem 4 every N samples */


/* ---- Line following: tunables ---- */
#define S3_PRI_LINE       17
#define S3_IR_MASK        0x5    /* GP26 left + GP28 right; use 0x7 ONLY with GP27 centre wired. */
#define JUNC_DARK         600   /* both sensors must be at least this dark (0..1000) */
#define JUNCTION_MIN_MS   30    /* ...for at least this long */


/* Timing.  LINE_PERIOD_MS MUST equal LINE_EVERY_N * S3_TICK_MS (checked below).
 * 2 ms gives a fast reaction to a sensor that only brushes the line edge.
 * Use 1/1 for the fastest possible loop, 10/10 for the old behaviour. */
#define S3_TICK_MS        1      /* barcode sampling tick */
#define LINE_EVERY_N      2      /* line-follow runs every Nth tick */
#define LINE_PERIOD_MS    2      /* = LINE_EVERY_N * S3_TICK_MS */
#if LINE_PERIOD_MS != (LINE_EVERY_N * S3_TICK_MS)
#error "LINE_PERIOD_MS must equal LINE_EVERY_N * S3_TICK_MS"
#endif


#define LINE_DARK         600   /* darkness (0..1000) that counts as "on black". Lower (400) = reacts earlier at the line edge. */
#define S3_CRUISE_RPM     30    /* about 0.105 m/s with 67 mm wheels */
#define S3_CURVE_RPM      24    /* outer wheel while steering */
#define S3_INNER_RPM      12    /* inner wheel: GENTLE level */
#define LEVEL_STOPPED     0     /* inner wheel: HARD level */


/* Escalation: time since the steer started (black time + seek time).
 *  0 .. S3_HARD_MS      gentle  : outer S3_CURVE_RPM,  inner S3_INNER_RPM
 *  S3_HARD_MS ..        hard    : outer S3_CURVE_RPM,  inner stopped
 *  S3_HARDER_MS ..      harder  : outer S3_HARDER_RPM, inner stopped
 * Timings are in ms and independent of LINE_PERIOD_MS. */
#define S3_HARD_MS        40
#define S3_HARDER_MS      120
#define S3_HARDER_RPM     S3_CRUISE_RPM   /* raise (36, 40) for a faster pivot; keep even */
/* SEEK: once a sensor has seen black, white/white is ambiguous: the line may
 * be back between the sensors OR it may have slid past the sensor to the
 * outside (curve too tight for the gentle turn).  Keep steering toward the
 * last black side, still escalating, until a sensor sees black again or this
 * many ms pass.  Then go straight (two-sensor mode).  0 = old "white = straight". */
#define S3_SEEK_MS        800


#define LOST_TIMEOUT_MS   1000  /* three-sensor mode only: stop after this long with no line */
#define S3_RESEND_MS      100


// Barcode
#define BC_ELEMENTS      29
#define S3_ADVANCE_MM    60     /* centre of wheels onto the junction: MEASURE sensor-to-axle distance */
#define S3_TURN_90       2      /* Buddy 2: steps of 45 degrees, so 2 = 90, 4 = 180 */


IMPORT void barcode_reset(void);
IMPORT void barcode_feed(INT dark);
IMPORT char barcode_take(void);
IMPORT BOOL barcode_busy(void);
IMPORT BOOL s3_motion_move_mm(INT mm);
IMPORT BOOL s3_motion_turn(INT dir, INT steps);   /* dir < 0 = left, > 0 = right */


/* Steering memory between calls of line_decide():
 *  last_pos : current steer direction, <0 left, >0 right, 0 straight
 *  dark_ms  : ms since this steer started (black + seek), drives escalation
 *  seek_ms  : remaining seek time after the sensor went white
 *  lost_ms  : three-sensor mode: how long no sensor has seen the line
 */
typedef struct { INT last_pos; INT dark_ms; INT seek_ms; INT lost_ms; } LF_STATE;
/* Wheel commands are RPM, not speed levels. */
typedef struct { INT left; INT right; BOOL stop; } LF_CMD;


IMPORT INT  ir_read(INT ch);
IMPORT ER   line_sense(INT *l, INT *c, INT *r);
IMPORT void line_decide(INT l, INT c, INT r, LF_STATE *st, LF_CMD *cmd);
IMPORT void line_follow_task(INT stacd, void *exinf);


/* Motion Control calls go through motion_adapter.c (prints now, real later) */
IMPORT void s3_motion_stop(void);
IMPORT void s3_motion_diagnostics(void);
IMPORT BOOL s3_motion_set_wheel_rpm(INT left, INT right);


/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID   mbx_samples;


/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *  tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
    UW  received;   /* samples received so far */
    INT min;
    INT max;
    W   sum;
} APP_STATS;


IMPORT ID       mtx_stats;
IMPORT APP_STATS    app_stats;


/* Private to subsystem 3 */
IMPORT void consumer_task(INT stacd, void *exinf);


#endif /* SUBSYSTEM3_H */