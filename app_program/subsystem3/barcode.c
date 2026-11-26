#include "barcode.h"
#include <stdbool.h>

#define BC_MIN_CONTRAST 160
#define BC_WHITE_FILTER 16
#define BC_ELEMENTS BARCODE_FRAME_ELEMENTS
#define BC_MIN_RUN_MS 15u
#define BC_MAX_RUN_MS 1100u
#define BC_PREFIX_SAMPLES 192u
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

/* Keeps the beginning of the label while the optical contrast settles.
   At the normal 10 ms cadence this covers the first wide black bar at
   any of the three print sizes. */
static struct {
    uint16_t elapsed_ms;
    int16_t contrast;
} prefix[BC_PREFIX_SAMPLES];

static struct {
    uint32_t sampled_at, edge_sampled_at, run_at, edge_at, prefix_at, first_at;
    uint32_t committed_bars, bars, width_rejects;
    uint16_t widths[BC_ELEMENTS];
    int white_level;
    uint16_t reference_peak, observed_peak, prefix_count, frame_gap_ms;
    int16_t first_contrast, previous_contrast;
    int8_t polarity, first_polarity;
    uint8_t count, missed_count;
    bool sampled, in_bar, edge_pending, have_edge, decoded, missed;
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

static bool starts_with_marker(const uint16_t *e)
{
    const C39 *symbol = decode_char(e);
    if (symbol && symbol->ch == '*') return true;
    uint16_t reverse[9];
    for (unsigned i = 0; i < 9u; ++i) reverse[i] = e[8u - i];
    symbol = decode_char(reverse);
    return symbol && symbol->ch == '*';
}

static void reset_edges(uint32_t now_ms)
{
    reader.edge_sampled_at = now_ms;
    reader.bars = 0;
    reader.in_bar = false;
    reader.edge_pending = false;
    reader.have_edge = false;
    reader.count = 0;
    reader.frame_gap_ms = 0;
    reader.previous_contrast = 0;
}

void barcode_reset(void)
{
    reader.committed_bars += reader.bars;
    reader.sampled = false;
    reader.polarity = 0;
    reader.first_polarity = 0;
    reader.prefix_count = 0;
    reader.reference_peak = 0;
    reader.observed_peak = 0;
    reader.decoded = reader.missed = false;
    reset_edges(0);
}

BARCODE_PROGRESS barcode_progress(void)
{
    return (BARCODE_PROGRESS){
        .bars = reader.committed_bars + reader.bars,
        .width_rejects = reader.width_rejects,
        .elements = reader.missed ? reader.missed_count : reader.count,
        .scan = reader.missed ? BARCODE_SCAN_MISSED :
                reader.polarity && !reader.decoded ? BARCODE_SCAN_ACTIVE : BARCODE_SCAN_IDLE,
    };
}

static char track_edge(int contrast, uint32_t now_ms)
{
    unsigned interval = now_ms - reader.edge_sampled_at;
    int previous = reader.previous_contrast;
    reader.edge_sampled_at = now_ms;
    reader.previous_contrast = (int16_t)contrast;

    int threshold = reader.reference_peak ? reader.reference_peak / 2 : BC_MIN_CONTRAST;
    bool crossed = reader.in_bar ? contrast < threshold : contrast > threshold;
    if (!crossed) {
        reader.edge_pending = false;
        return 0;
    }
    /* Interpolate the first threshold crossing and confirm it on the next
       sample so a single glitch cannot split a narrow bar or space. */
    if (!reader.edge_pending) {
        reader.edge_at = now_ms;
        int before = reader.in_bar ? previous - threshold : threshold - previous;
        int after = reader.in_bar ? threshold - contrast : contrast - threshold;
        if (before >= 0) {
            unsigned span = (unsigned)(before + after);
            reader.edge_at -= ((unsigned)after * interval + span / 2u) / span;
        }
        reader.edge_pending = true;
        return 0;
    }

    bool ended_bar = reader.in_bar;
    unsigned width = reader.edge_at - reader.run_at;
    reader.run_at = reader.edge_at;
    reader.in_bar = !ended_bar;
    reader.edge_pending = false;
    if (reader.in_bar) ++reader.bars;
    if (!reader.have_edge) {
        reader.have_edge = true;
        return 0; /* The leading white margin is outside the label. */
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
    if (ended_bar && reader.count == 9u && starts_with_marker(reader.widths)) {
        unsigned widest_space = 0;
        for (unsigned i = 1; i < 9u; i += 2u)
            if (reader.widths[i] > widest_space) widest_space = reader.widths[i];
        unsigned idle_ms = widest_space * 3u / 2u + BARCODE_MAX_SAMPLE_GAP_MS;
        reader.frame_gap_ms = (uint16_t)(idle_ms < BC_MAX_RUN_MS ? idle_ms : BC_MAX_RUN_MS);
    }
    if (!ended_bar || reader.count != BC_ELEMENTS) return 0;
    char letter = decode_any(reader.widths);
    if (letter) reader.count = 0;
    return letter;
}

static void save_prefix(int contrast, uint32_t now_ms)
{
    unsigned index = reader.prefix_count++;
    prefix[index].contrast = (int16_t)contrast;
    prefix[index].elapsed_ms = (uint16_t)(now_ms - reader.prefix_at);
}

static char track_sample(int contrast, uint32_t now_ms)
{
    if (reader.prefix_count < BC_PREFIX_SAMPLES) {
        save_prefix(contrast, now_ms);
        int previous = reader.previous_contrast;
        int supported = contrast < previous ? contrast : previous;
        if (supported > reader.observed_peak) reader.observed_peak = (uint16_t)supported;
        int peak = reader.observed_peak;
        /* Set the midpoint after a real peak has receded. A stronger wide bar
           can reveal that the earlier narrow runs never reached full black. */
        if (peak >= BC_MIN_CONTRAST && contrast <= peak - peak / 8 &&
            peak > reader.reference_peak + reader.reference_peak / 16) {
            reader.reference_peak = (uint16_t)peak;
            reset_edges(reader.prefix_at);
            char decoded = 0;
            for (unsigned i = 0; i < reader.prefix_count; ++i) {
                char letter = track_edge(prefix[i].contrast, reader.prefix_at + prefix[i].elapsed_ms);
                if (letter) decoded = letter;
            }
            return decoded;
        }
    }
    return track_edge(contrast, now_ms);
}

static void miss_scan(void)
{
    reader.missed_count = reader.count;
    barcode_reset();
    reader.missed = true;
}

char barcode_feed(int adc, uint32_t now_ms)
{
    /* A missing reading is neither black nor white. The next timestamp
       decides whether the gap was short enough to preserve this frame. */
    reader.missed = false;
    if (adc < 0 || adc > 4095) return 0;
    bool late = reader.sampled && now_ms - reader.sampled_at > BARCODE_MAX_SAMPLE_GAP_MS;
    bool in_frame = !reader.decoded && reader.frame_gap_ms && reader.count >= 11u;
    bool long_blank = !reader.in_bar && !reader.edge_pending &&
        now_ms - reader.run_at > reader.frame_gap_ms &&
        reader.polarity * (adc - reader.white_level / BC_WHITE_FILTER) < reader.reference_peak / 2;
    if (in_frame && (late || long_blank)) {
        /* Require a start marker and part of a payload before declaring a
           scan broken by a sample dropout or blank longer than a wide gap. */
        miss_scan();
        return 0;
    }
    if (!reader.sampled || late ||
        (reader.polarity && !reader.edge_pending && now_ms - reader.run_at > BC_MAX_RUN_MS)) {
        barcode_reset();
        reader.sampled = true;
        reader.sampled_at = now_ms;
        reader.white_level = adc * BC_WHITE_FILTER;
    }

    int delta = adc - reader.white_level / BC_WHITE_FILTER;
    uint32_t previous_at = reader.sampled_at;
    reader.sampled_at = now_ms;
    if (!reader.polarity) {
        int side = delta < 0 ? -1 : 1;
        int contrast = side * delta;
        if (contrast < BC_MIN_CONTRAST) {
            reader.white_level += delta;
            reader.first_polarity = 0;
            return 0;
        }
        if (side != reader.first_polarity) {
            reader.first_polarity = (int8_t)side;
            reader.first_contrast = (int16_t)contrast;
            reader.first_at = now_ms;
            reader.prefix_at = previous_at;
            return 0;
        }
        /* Freeze the paper level once the label starts. Blurred white gaps
           between two bars can sit far above the actual paper reading. */
        reader.polarity = (int8_t)side;
        reader.run_at = reader.prefix_at;
        reset_edges(reader.prefix_at);
        save_prefix(reader.first_contrast, reader.first_at);
        (void)track_edge(reader.first_contrast, reader.first_at);
    }

    uint32_t rejected = reader.width_rejects;
    char letter = track_sample(reader.polarity * delta, now_ms);
    if (letter) {
        reader.prefix_count = BC_PREFIX_SAMPLES;
        reader.decoded = true;
    } else if (!reader.decoded && reader.width_rejects != rejected) {
        miss_scan();
    }
    return letter;
}
