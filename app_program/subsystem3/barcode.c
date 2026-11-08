/* barcode.c: Code 39 scanner + decoder for the centre sensor.
   Feed it one darkness sample (0..1000) per tick. */
#include "subsystem3.h"

#define BC_TH_HIGH    600    /* darkness above this = bar   */
#define BC_TH_LOW     400    /* below this = space (hysteresis) */
#define BC_GLITCH       2    /* runs shorter than this are noise */
#define BC_MAX_RUN    100    /* longer than this cannot be part of a barcode */
#define BC_BUSY_TICKS  60    /* "scanning in progress" for this long after the last edge */

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

LOCAL BOOL in_bar;
LOCAL INT  run, nwin, idle, win[BC_ELEMENTS];
LOCAL char pending;

LOCAL char decode_char(const INT *w)            /* w[0..8]: bar, space, bar, ... */
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
LOCAL char decode29(const INT *e)
{
    char m;
    if (decode_char(e) != '*' || decode_char(e + 20) != '*') { return 0; }
    m = decode_char(e + 10);
    return (m == '*') ? 0 : m;
}

LOCAL char decode_any(const INT *e)
{
    INT rev[BC_ELEMENTS], i;
    char c = decode29(e);
    if (c != 0) { return c; }
    for (i = 0; i < BC_ELEMENTS; i++) { rev[i] = e[BC_ELEMENTS - 1 - i]; }   /* scanned backwards */
    return decode29(rev);
}

EXPORT void barcode_reset(void)
{
    in_bar = FALSE; run = 0; nwin = 0; pending = 0; idle = 10000;
}
EXPORT BOOL barcode_busy(void) { return (nwin >= 3 && idle < BC_BUSY_TICKS); }
EXPORT char barcode_take(void) { char c = pending; pending = 0; return c; }

EXPORT void barcode_feed(INT dark)
{
    BOOL bar = in_bar, was_bar;
    INT  i;
    char c;

    if (idle < 10000) { idle++; }
    if (!in_bar && dark > BC_TH_HIGH)    { bar = TRUE; }
    else if (in_bar && dark < BC_TH_LOW) { bar = FALSE; }

    if (bar == in_bar) { run++; return; }

    was_bar = in_bar;                           /* the element that just ended */
    if (run > BC_MAX_RUN || run < BC_GLITCH) {
        if (nwin >= 8) {
            tm_printf((UB*)"[S3] bc: reset after %d elements (run=%d)\n", nwin, run);
        }
        nwin = 0;
    } else {
        if (nwin == BC_ELEMENTS) {              /* slide the window by one */
            for (i = 1; i < BC_ELEMENTS; i++) { win[i - 1] = win[i]; }
            nwin--;
        }
        win[nwin++] = run;
        idle = 0;
        if (was_bar && nwin == BC_ELEMENTS) {   /* 29 ending in a bar = starts with a bar */
            c = decode_any(win);
            if (c != 0) { pending = c; nwin = 0; }
            else {
                tm_printf((UB*)"[S3] bc: 29 elements, no match:");
                for (i = 0; i < BC_ELEMENTS; i++) { tm_printf((UB*)" %d", win[i]); }
                tm_printf((UB*)"\n");
            }
        }
    }
    in_bar = bar;
    run = 1;
}