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
/* Both outer sensors dark for at least this long = cross junction.
 * MUST be longer than the widest barcode bar takes to pass (bar_mm / 0.105 mm/ms
 * at 30 RPM: a 6 mm bar = ~57 ms) and shorter than the crossing line
 * (20 mm = ~190 ms).  The first bars of a barcode are not yet "busy". */
#define JUNCTION_MIN_MS   100
/* After a junction action, ignore junction detection for this long so the
 * rest of the crossing line (or the line after a turn) cannot retrigger it. */
#define JUNC_BLANK_MS     600
/* What to do at a junction when no barcode was read, or the letter is not in
 * the mapping:  ACT_STRAIGHT = drive on (testing / robust demo)
 *               ACT_NONE     = stop and wait until a mapping resolves it */
#define S3_JUNCTION_DEFAULT  ACT_STRAIGHT


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


// Barcode (Code 39, * X *, read on the centre sensor GP27 every S3_TICK_MS)
#define BC_ELEMENTS      29
/* Detection happens JUNCTION_MIN_MS into the crossing line (~10 mm), so
   ADVANCE = (sensor-to-axle distance) - 10 mm.  MEASURE on the car. */
#define S3_ADVANCE_MM    60
#define S3_TURN_90       2      /* Buddy 2: steps of 45 degrees, so 2 = 90, 4 = 180 */


/* The scanner consumes ONE BIT per tick: 1 = bar (black), 0 = space (white).
 * Source is selectable:
 *   AO (default): centre ADC -> darkness -> 1 bit with BC_TH_HIGH/BC_TH_LOW hysteresis
 *   DO: define S3_BC_DO_PIN to the GPIO wired to the module's DO pin; the
 *       comparator on the module does the thresholding (trim pot!).
 * The decoder is identical either way. */
#define S3_BC_DO_PIN   27          /* <- uncomment + set GPIO to use DO */
#define S3_BC_DO_ACTIVE_LOW 0           /* most modules: DO HIGH over black. Set 1 if yours is inverted */
#define BC_TH_HIGH    600               /* AO path: darkness above this = bar   */
#define BC_TH_LOW     400               /* AO path: below this = space (hysteresis) */

/* Bench testing without the MQTT task: define a 4-letter mapping (F U L R)
 * and the task loads it at start-up, so junction actions work and
 * "[S3] barcode 'X' stored -> ACTION" names a real action.  REMOVE for the
 * demo (the mapping must come from MQTT). */
#define S3_TEST_MAP   "QAFZ"   /* Forward=Q, U-turn=A, Left=F, Right=Z */

/* TEST ONLY: print "1" when the centre sensor sees black and "0" when it
 * sees white again (on change, so it costs nothing while the state holds).
 * Use it to check sensor height / trim pot / DO polarity.  Leave undefined
 * for normal runs. */
/* #define S3_PRINT_CENTRE_BIT */

/* TEST ONLY: drive the motors straight from this task (s3_motion_*) instead
 * of going through the vehicle controller.  If the unexplained stops vanish
 * with this defined, the stop is coming from the controller or another
 * subsystem that calls the motion API, not from subsystem 3. */
/* #define S3_DIRECT_MOTION */

IMPORT void barcode_reset(void);
IMPORT void barcode_feed(BOOL bar);
IMPORT char barcode_take(void);         /* 'A'..'Z' or 0 */
IMPORT BOOL barcode_busy(void);
IMPORT void barcode_diag(void);         /* prints deferred scanner diagnostics; call at low rate */
IMPORT BOOL ir_read_do(INT gpio);       /* raw digital input of one GPIO (DO path) */
IMPORT BOOL s3_motion_move_mm(INT mm);
IMPORT BOOL s3_motion_turn(INT dir, INT steps);   /* dir < 0 = left, > 0 = right */


/* ---- IF-05: barcode character -> junction action, delivered over MQTT ----
 * The mapping is NOT hard-coded.  Whoever owns the MQTT client calls
 *     s3_route_parse(payload, len)
 * from its receive task (any task; not from an ISR) with the raw payload.
 *
 * Primary payload: 3 bytes, compact binary (Change Requirement 1).
 * Letters are coded 1..26 (A=1), 5 bits each, big-endian bit order:
 *     bits 23..19 forward | 18..14 uturn | 13..9 left | 8..4 right | 3..0 check
 *     check = (forward ^ uturn ^ left ^ right) & 0xF
 * e.g. A,B,C,F -> F=1 U=2 L=3 R=6 check=6 -> 0x08 0x86 0x66
 * Debug fallback (text, any other length): {"forward":"A",...}, F=A U=B L=C R=F, or ABCF.
 *
 * Storage: one 32-bit word, 4 x 5 bits.  Written whole by the MQTT task, read
 * whole by the line task: single aligned word access is atomic on Cortex-M,
 * so there is no read-modify-write on shared packed data and no lock.
 * Until a valid mapping has arrived, the car stops at the first junction and
 * waits; it carries on as soon as the mapping is received.
 */
typedef enum { ACT_NONE = 0, ACT_STRAIGHT, ACT_UTURN, ACT_LEFT, ACT_RIGHT } S3_ACTION;  /* 5 values: 3 bits */
#define S3_ROUTE_BIN_LEN 3
IMPORT INT       s3_route_encode(char forward, char uturn, char left, char right, UB out[S3_ROUTE_BIN_LEN]); /* PC-side reference / tests */

IMPORT ER        s3_route_parse(const UB *payload, INT len);
IMPORT ER        s3_route_set_map(char forward, char uturn, char left, char right);
IMPORT BOOL      s3_route_valid(void);
IMPORT S3_ACTION s3_route_lookup(char code);            /* ACT_NONE if unmapped / no mapping */
IMPORT void      s3_route_get(char out4[4]);            /* current mapping F,U,L,R ('-' = unset) */
IMPORT const char *s3_action_name(S3_ACTION a);


/* ---- Minimum-bit data representation (Change Requirement 1) ----------
 * Persistent and shared application data is packed to the bits its valid
 * range needs (C bit-fields).  Function parameters and short-lived locals
 * are native INT: they are register temporaries, not stored data.
 * Table, sizes and evidence: S3_DATA_REPRESENTATION.md
 */
#define DIR_NONE   0
#define DIR_LEFT   1
#define DIR_RIGHT  2

/* Steering memory between calls of line_decide(): one 32-bit word. */
typedef struct {
    UW dir     : 2;   /* DIR_NONE / DIR_LEFT / DIR_RIGHT                        */
    UW dark_ms : 10;  /* ms since this steer started (black + seek), saturates  */
    UW seek_ms : 10;  /* remaining seek time after the sensor went white        */
    UW lost_ms : 10;  /* three-sensor mode: ms since any sensor saw the line    */
} LF_STATE;
#define LF_MS_MAX  1023
#if (S3_SEEK_MS > LF_MS_MAX) || (LOST_TIMEOUT_MS > LF_MS_MAX) || (S3_HARDER_MS >= LF_MS_MAX)
#error "LF_STATE ms fields are 10 bits: keep S3_SEEK_MS, LOST_TIMEOUT_MS <= 1023"
#endif

/* Wheel command: RPM per wheel (0..63) + stop flag, 13 bits in a half-word. */
typedef struct {
    UH left  : 6;
    UH right : 6;
    UH stop  : 1;
} LF_CMD;
#if (S3_CRUISE_RPM > 63) || (S3_HARDER_RPM > 63)
#error "LF_CMD RPM fields are 6 bits (0..63)"
#endif

/* One line-sensor sample: three darkness values 0..1000 (10 bits each). */
typedef struct {
    UW l : 10;
    UW c : 10;
    UW r : 10;
    UW ok : 1;        /* 0 = ADC error this cycle */
} LF_SENSE;


IMPORT INT  ir_read(INT ch);
IMPORT LF_SENSE line_sense(void);
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