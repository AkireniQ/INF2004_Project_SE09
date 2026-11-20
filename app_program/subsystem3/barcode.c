/* barcode.c: Code 39 scanner + decoder for the centre sensor.
   Feed it ONE BIT per tick (1 = bar/black, 0 = space/white), from the AO
   path (darkness + hysteresis) or straight from the module's DO pin.
   Decodes * X * for X in A..Z, scanned in either direction.

   Data representation (Change Requirement 1): all scanner state lives in
   one packed 32-bit word plus 29 one-byte element widths. */
#include "subsystem3.h"

#define BC_GLITCH       2    /* runs shorter than this are noise (merged into the neighbour) */
#define BC_MAX_RUN    100    /* longer than this cannot be part of a barcode (ticks) */
#define BC_BUSY_TICKS  60    /* "scanning in progress" for this long after the last edge */
#define BC_RUN_MAX    127    /* 7-bit saturation, > BC_MAX_RUN so the reset test still fires */
#if BC_MAX_RUN > 255
#error "element widths are stored in UB"
#endif

typedef struct { char ch; const char *pat; } C39;   /* w = wide, n = narrow, bar/space alternating */
LOCAL const C39 c39[] = {
    {'A',"wnnnnwnnw"},{'B',"nnwnnwnnw"},{'C',"wnwnnwnnn"},{'D',"nnnnwwnnw"},
    {'E',"wnnnwwnnn"},{'F',"nnwnwwnnn"},{'G',"nnnnnwwnw"},{'H',"wnnnnwwnn"},
    {'I',"nnwnnwwnn"},{'J',"nnnnwwwnn"},{'K',"wnnnnnnww"},{'L',"nnwnnnnww"},
    {'M',"wnwnnnnwn"},{'N',"nnnnwnnww"},{'O',"wnnnwnnwn"},{'P',"nnwnwnnwn"},
    {'Q',"nnnnnnwww"},{'R',"wnnnnnwwn"},{'S',"nnwnnnwwn"},{'T',"nnnnwnwwn"},
    {'U',"wwnnnnnnw"},{'V',"nwwnnnnnw"},{'W',"wwwnnnnnn"},{'X',"nwnnwnnnw"},
    {'Y',"wwnnwnnnn"},{'Z',"nwwnwnnnn"},{'*',"nwnnwnwnn"}
};

/* Scanner state: 1 + 7 + 5 + 7 + 5 + 7 = 32 bits. */
LOCAL struct {
    UW in_bar   : 1;    /* current element type                          */
    UW run      : 7;    /* ticks in the current element, saturates at 127 */
    UW nwin     : 5;    /* elements in the window, 0..29                 */
    UW idle     : 7;    /* ticks since the last edge, saturates at 127   */
    UW pending  : 5;    /* decoded letter 1..26 (A=1), 0 = none          */
    UW glitches : 7;    /* merged blips since the last barcode_diag()    */
} bc;
LOCAL UB win[BC_ELEMENTS];            /* element widths in ticks, each <= BC_MAX_RUN */

/* Deferred diagnostics: barcode_feed() runs inside the 1 ms tick, where a
   tm_printf of 29 numbers (~100 ms on the UART) would wreck line following. */
LOCAL UB   diag_fail[BC_ELEMENTS];
LOCAL struct {
    UW fail_pending : 1;
    UW resets       : 7;
    UW reset_n      : 5;
    UW reset_run    : 7;
} diag;

LOCAL char decode_char(const UB *w)             /* w[0..8]: bar, space, bar, ... */
{
    BOOL wide[9];
    INT i, k, best, wb = 32767, nb = 0, ws, ns = 0;

    for (i = 0; i < 9; i++) { wide[i] = FALSE; }
    for (k = 0; k < 2; k++) {                   /* the 2 widest bars */
        best = -1;
        for (i = 0; i < 9; i += 2) {
            if (!wide[i] && (best < 0 || w[i] > w[best])) { best = i; }
        }
        wide[best] = TRUE;
    }
    best = -1;                                  /* the widest space */
    for (i = 1; i < 9; i += 2) {
        if (best < 0 || w[i] > w[best]) { best = i; }
    }
    wide[best] = TRUE;
    ws = w[best];

    for (i = 0; i < 9; i += 2) {                /* bars: wide must beat narrow */
        if (wide[i]) { if (w[i] < wb) { wb = w[i]; } }
        else         { if (w[i] > nb) { nb = w[i]; } }
    }
    for (i = 1; i < 9; i += 2) {
        if (!wide[i] && w[i] > ns) { ns = w[i]; }
    }
    if (wb * 10 < nb * 13 || ws * 10 < ns * 13) { return 0; }

    for (k = 0; k < (INT)(sizeof(c39) / sizeof(c39[0])); k++) {
        for (i = 0; i < 9; i++) {
            if ((c39[k].pat[i] == 'w') != wide[i]) { break; }
        }
        if (i == 9) { return c39[k].ch; }
    }
    return 0;
}

/* 29 elements = * (9) gap (1) X (9) gap (1) * (9) */
LOCAL char decode29(const UB *e)
{
    char m;
    if (decode_char(e) != '*' || decode_char(e + 20) != '*') { return 0; }
    m = decode_char(e + 10);
    return (m == '*') ? 0 : m;
}

LOCAL char decode_any(const UB *e)
{
    UB rev[BC_ELEMENTS];
    INT i;
    char c = decode29(e);
    if (c != 0) { return c; }
    for (i = 0; i < BC_ELEMENTS; i++) { rev[i] = e[BC_ELEMENTS - 1 - i]; }   /* scanned backwards */
    return decode29(rev);
}

EXPORT void barcode_reset(void)
{
    bc.in_bar = 0; bc.run = 0; bc.nwin = 0; bc.pending = 0; bc.idle = BC_RUN_MAX;
}
EXPORT BOOL barcode_busy(void) { return (bc.nwin >= 3 && bc.idle < BC_BUSY_TICKS); }
EXPORT char barcode_take(void)
{
    UW p = bc.pending;
    bc.pending = 0;
    return p ? (char)('A' + p - 1) : 0;
}

EXPORT void barcode_diag(void)
{
    INT i;
    if (diag.fail_pending) {
        diag.fail_pending = 0;
        tm_printf((UB*)"[S3] bc: 29 elements, no match:");
        for (i = 0; i < BC_ELEMENTS; i++) { tm_printf((UB*)" %d", (INT)diag_fail[i]); }
        tm_printf((UB*)"\n");
    }
    if (diag.resets || bc.glitches) {
        tm_printf((UB*)"[S3] bc: %d window reset(s), last after %d elements (run=%d), %d glitches merged\n",
                  (INT)diag.resets, (INT)diag.reset_n, (INT)diag.reset_run, (INT)bc.glitches);
        diag.resets = 0; bc.glitches = 0;
    }
}

EXPORT void barcode_feed(BOOL bar)
{
    INT  i, run;
    char c;

    bar = bar ? 1 : 0;
    if (bc.idle < BC_RUN_MAX) { bc.idle++; }

    if (bar == (BOOL)bc.in_bar) {
        if (bc.run < BC_RUN_MAX) { bc.run++; }
        return;
    }

    run = bc.run;                               /* the element that just ended */

    if (run < BC_GLITCH && bc.nwin > 0) {
        /* A 1-tick blip inside an element: fold it back into the element it
           interrupted and carry on with that element instead of resetting. */
        if (bc.glitches < 127) { bc.glitches++; }
        bc.nwin--;
        run += win[bc.nwin] + 1;
        bc.run = (run > BC_RUN_MAX) ? BC_RUN_MAX : run;
        bc.in_bar = bar;                        /* back to the interrupted element's type */
        return;
    }

    if (run > BC_MAX_RUN || run < BC_GLITCH) {
        if (bc.nwin >= 8) {
            if (diag.resets < 127) { diag.resets++; }
            diag.reset_n = bc.nwin; diag.reset_run = run;
        }
        bc.nwin = 0;
    } else {
        if (bc.nwin == BC_ELEMENTS) {           /* slide the window by one */
            for (i = 1; i < BC_ELEMENTS; i++) { win[i - 1] = win[i]; }
            bc.nwin--;
        }
        win[bc.nwin++] = (UB)run;
        bc.idle = 0;
        if (bc.in_bar && bc.nwin == BC_ELEMENTS) {   /* 29 ending in a bar = starts with a bar */
            c = decode_any(win);
            if (c != 0) { bc.pending = (UW)(c - 'A' + 1); bc.nwin = 0; }
            else if (!diag.fail_pending) {
                for (i = 0; i < BC_ELEMENTS; i++) { diag_fail[i] = win[i]; }
                diag.fail_pending = 1;
            }
        }
    }
    bc.in_bar = bar;
    bc.run = 1;
}