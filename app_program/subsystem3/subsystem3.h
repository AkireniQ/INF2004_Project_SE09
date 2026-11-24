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


/* Left and right steer. The center channel independently reads barcodes. */
#include "line_steering.h"
#define S3_PRI_LINE       12
#define LINE_PERIOD_MS    10
#define BC_ELEMENTS       29

IMPORT void barcode_reset(void);
IMPORT void barcode_feed(INT dark);
IMPORT char barcode_take(void);
IMPORT BOOL barcode_busy(void);
IMPORT BOOL s3_motion_move_mm(INT mm);
IMPORT BOOL s3_motion_turn(INT dir, INT steps);   /* dir < 0 = left, > 0 = right */


IMPORT INT  ir_read(INT ch);
IMPORT ER   line_sense(INT *l, INT *c, INT *r);
IMPORT void line_follow_task(INT stacd, void *exinf);
IMPORT void s3_log_sampler_state(void);
#if APP_IR_BENCH
IMPORT void ir_bench_run(void);
#endif


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
