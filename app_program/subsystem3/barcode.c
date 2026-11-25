#include "barcode.h"
#include <stdbool.h>

#define BC_MIN_CONTRAST 160
#define BC_WHITE_FILTER 16
#define BC_ELEMENTS BARCODE_FRAME_ELEMENTS
#define BC_MIN_RUN_MS 15u
#define BC_MAX_RUN_MS 1100u
#define BC_SENTINEL_PATTERN "nwnnwnwnn"

typedef struct { char ch; const char *pat; } C39;   /* w = wide, n = narrow, bar/space alternating */
static const C39 c39[] = {
    {'A',"wnnnnwnnw"},{'B',"nnwnnwnnw"},{'C',"wnwnnwnnn"},{'D',"nnnnwwnnw"},
    {'E',"wnnnwwnnn"},{'F',"nnwnwwnnn"},{'G',"nnnnnwwnw"},{'H',"wnnnnwwnn"},
    {'I',"nnwnnwwnn"},{'J',"nnnnwwwnn"},{'K',"wnnnnnnww"},{'L',"nnwnnnnww"},
    {'M',"wnwnnnnwn"},{'N',"nnnnwnnww"},{'O',"wnnnwnnwn"},{'P',"nnwnwnnwn"},
    {'Q',"nnnnnnwww"},{'R',"wnnnnnwwn"},{'S',"nnwnnnwwn"},{'T',"nnnnwnwwn"},
    {'U',"wwnnnnnnw"},{'V',"nwwnnnnnw"},{'W',"wwwnnnnnn"},{'X',"nwnnwnnnw"},
    {'Y',"wwnnwnnnn"},{'Z',"nwwnwnnnn"},{'*',BC_SENTINEL_PATTERN}
};

static struct {
    uint32_t sampled_at, run_at, edge_at;
    uint32_t width_rejects;
    uint16_t widths[BC_ELEMENTS];
    int white_level;
    uint16_t bar_peak;
    int16_t previous_contrast;
    int8_t polarity, edge_side;
    uint8_t count;
    bool sampled, in_bar, edge_pending;
} reader;

static const C39 *decode_char(const uint16_t *w)  /* w[0..8]: bar, space, bar, ... */
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
        if (i == 9) { return &c39[k]; }
    }
    return 0;
}

/* 29 elements = * (9) gap (1) X (9) gap (1) * (9) */
static bool character_gap(const uint16_t *sentinel, unsigned gap)
{
    unsigned narrow = (sentinel[3] + sentinel[5] + sentinel[7]) / 3u;
    return gap * 2u >= narrow && gap <= 2u * narrow;
}

static bool payload_widths_match(const uint16_t *e, const char *pattern)
{
    /* The two sentinels calibrate bar and space widths separately because
       the optical footprint and edge thresholds affect them differently. */
    const char *sentinel_pattern = BC_SENTINEL_PATTERN;
    unsigned widths[2][2] = {{0}};
    unsigned counts[2][2] = {{0}};
    for (unsigned i = 0; i < 9; ++i) {
        unsigned space = i % 2u;
        bool wide = sentinel_pattern[i] == 'w';
        widths[space][wide] += e[i] + e[20 + i];
        counts[space][wide] += 2u;
    }

    for (unsigned i = 0; i < 9; ++i) {
        unsigned space = i % 2u;
        bool wide = pattern[i] == 'w';
        unsigned measured = e[10 + i] * counts[space][wide] * 100u;
        unsigned reference = widths[space][wide];
        /* Rank alone can promote a stretched narrow bar and demote a
           shortened wide bar, turning F into I or another valid symbol. */
        unsigned minimum = wide ? 77u : 50u;
        unsigned maximum = wide ? 140u : 160u;
        if (measured < reference * minimum || measured > reference * maximum) return false;
    }
    return true;
}

static char decode29(const uint16_t *e, bool *width_rejected)
{
    const C39 *start = decode_char(e);
    if (!start || start->ch != '*') return 0;
    const C39 *end = decode_char(e + 20);
    if (!end || end->ch != '*' ||
        !character_gap(e, e[9]) || !character_gap(e + 20, e[19])) {
        return 0;
    }
    const C39 *letter = decode_char(e + 10);
    if (!letter || letter->ch == '*') return 0;
    if (!payload_widths_match(e, letter->pat)) {
        *width_rejected = true;
        return 0;
    }
    return letter->ch;
}

static char decode_any(const uint16_t *e)
{
    uint16_t rev[BC_ELEMENTS];
    bool width_rejected = false;
    char c = decode29(e, &width_rejected);
    if (c != 0) { return c; }
    for (unsigned i = 0; i < BC_ELEMENTS; ++i) { rev[i] = e[BC_ELEMENTS - 1 - i]; }
    c = decode29(rev, &width_rejected);
    if (!c && width_rejected) ++reader.width_rejects;
    return c;
}

void barcode_reset(void)
{
    reader.sampled = false;
    reader.in_bar = false;
    reader.edge_pending = false;
    reader.count = 0;
    reader.polarity = 0;
    reader.bar_peak = 0;
    reader.previous_contrast = 0;
}

BARCODE_PROGRESS barcode_progress(void)
{
    return (BARCODE_PROGRESS){
        .width_rejects = reader.width_rejects,
        .elements = reader.count,
        .in_bar = reader.in_bar,
    };
}

char barcode_feed(int adc, uint32_t now_ms)
{
    /* A missing reading is neither black nor white. The next timestamp
       decides whether the gap was short enough to preserve this frame. */
    if (adc < 0 || adc > 4095) return 0;
    if (!reader.sampled || now_ms - reader.sampled_at > BARCODE_MAX_SAMPLE_GAP_MS ||
        (reader.in_bar && !reader.edge_pending && now_ms - reader.run_at > BC_MAX_RUN_MS)) {
        barcode_reset();
        reader.sampled = true;
        reader.run_at = now_ms;
        reader.white_level = adc * BC_WHITE_FILTER;
    }
    reader.sampled_at = now_ms;

    if (!reader.in_bar && !reader.edge_pending && now_ms - reader.run_at > BC_MAX_RUN_MS) {
        /* A new label can be on paper with a different reflectance. */
        reader.polarity = 0;
        reader.bar_peak = 0;
        reader.count = 0;
    }

    int delta = adc - reader.white_level / BC_WHITE_FILTER;
    int side = reader.polarity;
    if (!side) side = delta < 0 ? -1 : 1;
    int contrast = side * delta;
    int threshold = reader.in_bar ? reader.bar_peak * 2 / 5 : reader.bar_peak / 4;
    int minimum = reader.in_bar ? BC_MIN_CONTRAST / 2 : BC_MIN_CONTRAST;
    if (threshold < minimum) threshold = minimum;
    bool crossed = reader.in_bar ? contrast <= threshold : contrast >= threshold;
    /* Only use peaks supported by two readings, so a spike doesn't move
       the threshold for the rest of the bar. */
    int supported = contrast < reader.previous_contrast ? contrast : reader.previous_contrast;
    reader.previous_contrast = (int16_t)contrast;
    if (!crossed) {
        reader.edge_pending = false;
        if (!reader.in_bar) reader.white_level += delta;
        else if (supported > reader.bar_peak) reader.bar_peak = (uint16_t)supported;
        return 0;
    }
    /* Timestamp the first sample of a confirmed edge. One bad sample
       within a bar or space does not split its width. */
    if (!reader.edge_pending || reader.edge_side != side) {
        reader.edge_at = now_ms;
        reader.edge_side = (int8_t)side;
        reader.previous_contrast = (int16_t)contrast;
        reader.edge_pending = true;
        return 0;
    }

    bool ended_bar = reader.in_bar;
    unsigned width = reader.edge_at - reader.run_at;
    reader.run_at = reader.edge_at;
    reader.in_bar = !ended_bar;
    reader.edge_pending = false;
    if (!ended_bar) {
        reader.polarity = (int8_t)side;
        reader.bar_peak = (uint16_t)supported;
    }
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
