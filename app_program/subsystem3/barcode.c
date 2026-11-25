#include "barcode.h"
#include <stdbool.h>

#define BC_TH_HIGH 600
#define BC_TH_LOW 400
#define BC_ELEMENTS BARCODE_FRAME_ELEMENTS
#define BC_MIN_RUN_MS 15u
#define BC_MAX_RUN_MS 1100u

typedef struct { char ch; const char *pat; } C39;   /* w = wide, n = narrow, bar/space alternating */
static const C39 c39[] = {
    {'A',"wnnnnwnnw"},{'B',"nnwnnwnnw"},{'C',"wnwnnwnnn"},{'D',"nnnnwwnnw"},
    {'E',"wnnnwwnnn"},{'F',"nnwnwwnnn"},{'G',"nnnnnwwnw"},{'H',"wnnnnwwnn"},
    {'I',"nnwnnwwnn"},{'J',"nnnnwwwnn"},{'K',"wnnnnnnww"},{'L',"nnwnnnnww"},
    {'M',"wnwnnnnwn"},{'N',"nnnnwnnww"},{'O',"wnnnwnnwn"},{'P',"nnwnwnnwn"},
    {'Q',"nnnnnnwww"},{'R',"wnnnnnwwn"},{'S',"nnwnnnwwn"},{'T',"nnnnwnwwn"},
    {'U',"wwnnnnnnw"},{'V',"nwwnnnnnw"},{'W',"wwwnnnnnn"},{'X',"nwnnwnnnw"},
    {'Y',"wwnnwnnnn"},{'Z',"nwwnwnnnn"},{'*',"nwnnwnwnn"}
};

static struct {
    uint32_t sampled_at, run_at, edge_at;
    uint16_t widths[BC_ELEMENTS];
    uint8_t count;
    bool sampled, in_bar, edge_pending;
} reader;

static char decode_char(const uint16_t *w)      /* w[0..8]: bar, space, bar, ... */
{
    bool wide[9] = {0};
    int i, k, best, wb = 32767, nb = 0, ws, ns = 0;

    for (k = 0; k < 2; k++) {                   /* the 2 widest bars */
        best = -1;
        for (i = 0; i < 9; i += 2) {
            if (!wide[i] && (best < 0 || w[i] > w[best])) { best = i; }
        }
        wide[best] = true;
    }
    best = -1;                                  /* the widest space */
    for (i = 1; i < 9; i += 2) {
        if (best < 0 || w[i] > w[best]) { best = i; }
    }
    wide[best] = true;
    ws = w[best];

    for (i = 0; i < 9; i += 2) {                /* bars: wide must beat narrow */
        if (wide[i]) { if (w[i] < wb) { wb = w[i]; } }
        else         { if (w[i] > nb) { nb = w[i]; } }
    }
    for (i = 1; i < 9; i += 2) {
        if (!wide[i] && w[i] > ns) { ns = w[i]; }
    }
    if (wb * 10 < nb * 13 || ws * 10 < ns * 13) { return 0; }

    for (k = 0; k < (int)(sizeof(c39) / sizeof(c39[0])); k++) {
        for (i = 0; i < 9; i++) {
            if ((c39[k].pat[i] == 'w') != wide[i]) { break; }
        }
        if (i == 9) { return c39[k].ch; }
    }
    return 0;
}

/* 29 elements = * (9) gap (1) X (9) gap (1) * (9) */
static bool character_gap(const uint16_t *sentinel, unsigned gap)
{
    unsigned narrow = (sentinel[3] + sentinel[5] + sentinel[7]) / 3u;
    return gap * 2u >= narrow && gap <= 2u * narrow;
}

static char decode29(const uint16_t *e)
{
    if (decode_char(e) != '*' || decode_char(e + 20) != '*' ||
        !character_gap(e, e[9]) || !character_gap(e + 20, e[19])) {
        return 0;
    }
    char letter = decode_char(e + 10);
    return letter == '*' ? 0 : letter;
}

static char decode_any(const uint16_t *e)
{
    uint16_t rev[BC_ELEMENTS];
    char c = decode29(e);
    if (c != 0) { return c; }
    for (unsigned i = 0; i < BC_ELEMENTS; ++i) { rev[i] = e[BC_ELEMENTS - 1 - i]; }
    return decode29(rev);
}

void barcode_reset(void)
{
    reader.sampled = false;
    reader.in_bar = false;
    reader.edge_pending = false;
    reader.count = 0;
}

BARCODE_PROGRESS barcode_progress(void)
{
    return (BARCODE_PROGRESS){.elements = reader.count, .in_bar = reader.in_bar};
}

char barcode_feed(int darkness, uint32_t now_ms)
{
    if (!reader.sampled || now_ms - reader.sampled_at > BARCODE_MAX_SAMPLE_GAP_MS) {
        barcode_reset();
        reader.sampled = true;
        reader.run_at = now_ms;
    }
    reader.sampled_at = now_ms;

    bool crossed = reader.in_bar ? darkness < BC_TH_LOW : darkness > BC_TH_HIGH;
    if (!crossed) {
        reader.edge_pending = false;
        return 0;
    }
    /* Timestamp the first sample of a confirmed edge. One bad sample
       within a bar or space does not split its width. */
    if (!reader.edge_pending) {
        reader.edge_at = now_ms;
        reader.edge_pending = true;
        return 0;
    }

    bool ended_bar = reader.in_bar;
    unsigned width = reader.edge_at - reader.run_at;
    reader.run_at = reader.edge_at;
    reader.in_bar = !ended_bar;
    reader.edge_pending = false;
    if (width < BC_MIN_RUN_MS || width > BC_MAX_RUN_MS) {
        reader.count = 0;
        return 0;
    }
    if (reader.count == BC_ELEMENTS) {
        for (unsigned i = 1; i < BC_ELEMENTS; ++i) {
            reader.widths[i - 1] = reader.widths[i];
        }
        --reader.count;
    }
    reader.widths[reader.count++] = (uint16_t)width;
    if (!ended_bar || reader.count != BC_ELEMENTS) return 0;
    char letter = decode_any(reader.widths);
    if (letter) reader.count = 0;
    return letter;
}
