/*
 *	subsystem3.h
 *	Subsystem 3 -- Consumer (statistics).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"

#define S3_PRI_CONSUMER		16	/* priority band 16-17 (TEAM.md) */
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* ---- Line following: tunables ---- */
#define S3_PRI_LINE       17
#define S3_IR_MASK   0x1    /* TEST: left sensor only.   Use 0x4 for right only. Car: 0x5 */
#define JUNC_DARK         600   /* both sensors must be at least this dark (0..1000) */
#define JUNCTION_MIN_MS   30    /* ...for at least this long */

#define LINE_PERIOD_MS    10
#define POS_STRAIGHT      150
#define POS_GENTLE        450
#define LEVEL_CRUISE      2
#define LEVEL_SLOW        1
#define LEVEL_STOPPED     0
#define LOST_SUM_MAX      150
#define LOST_TIMEOUT_MS   1000
#define S3_RESEND_MS   100

typedef struct { INT last_pos; INT lost_ms; } LF_STATE;
typedef struct { INT left; INT right; BOOL stop; } LF_CMD;

IMPORT INT  ir_read(INT ch);
IMPORT ER   line_sense(INT *l, INT *c, INT *r);
IMPORT void line_decide(INT l, INT c, INT r, LF_STATE *st, LF_CMD *cmd);
IMPORT void line_follow_task(INT stacd, void *exinf);

/* Motion Control calls go through motion_adapter.c (prints now, real later) */
IMPORT void s3_motion_stop(void);
IMPORT BOOL s3_motion_set_wheel_levels(INT left, INT right);

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

/* Private to subsystem 3 */
IMPORT void	consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
