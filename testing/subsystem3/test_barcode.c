#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../../app_program/subsystem3/barcode.h"

/* BARCODE (1).pptx. Each letter has large, medium and small slides. */
static const struct {
    char letter;
    const char *elements;
} samples[] = {
    {'A', "nwnnwnwnn" "n" "wnnnnwnnw" "n" "nwnnwnwnn"},
    {'F', "nwnnwnwnn" "n" "nnwnwwnnn" "n" "nwnnwnwnn"},
    {'I', "nwnnwnwnn" "n" "nnwnnwwnn" "n" "nwnnwnwnn"},
    {'Z', "nwnnwnwnn" "n" "nwwnwnnnn" "n" "nwnnwnwnn"},
};

static unsigned reflected_shade(const unsigned *bounds, unsigned position, unsigned half_spot)
{
    unsigned lo = position > half_spot ? position - half_spot : 0;
    unsigned hi = position + half_spot;
    unsigned dark = 0;
    for (unsigned i = 0; i < BARCODE_FRAME_ELEMENTS; i += 2u) {
        unsigned start = bounds[i] > lo ? bounds[i] : lo;
        unsigned end = bounds[i + 1u] < hi ? bounds[i + 1u] : hi;
        if (end > start) dark += end - start;
    }
    return dark * 256u / (2u * half_spot);
}

static void check_print_size(unsigned narrow_um, unsigned half_spot)
{
    for (unsigned letter = 0; letter < sizeof(samples) / sizeof(samples[0]); ++letter) {
        for (unsigned reverse = 0; reverse < 2; ++reverse) {
            unsigned bounds[BARCODE_FRAME_ELEMENTS + 1u] = {23000u};
            for (unsigned i = 0; i < BARCODE_FRAME_ELEMENTS; ++i) {
                unsigned j = reverse ? BARCODE_FRAME_ELEMENTS - 1u - i : i;
                unsigned scale = samples[letter].elements[j] == 'w' ? 3u : 1u;
                bounds[i + 1u] = bounds[i] + narrow_um * scale;
            }

            for (unsigned speed = 34; speed <= 50; speed += 8u) {
                unsigned reads = 0;
                uint32_t now = 0;
                barcode_reset();
                uint32_t initial_bars = barcode_progress().bars;
                for (unsigned position = 0; position < bounds[BARCODE_FRAME_ELEMENTS] + 20000u; ) {
                    /* A narrow bar or gap can sit entirely inside the sensor's
                       footprint, so it never reaches a pure black or white. */
                    unsigned shade = reflected_shade(bounds, position, half_spot);
                    int adc = 220 + (int)(3280u * shade / 256u) + (int)(now / 10u % 7u) * 6 - 18;
                    char decoded = barcode_feed(adc, now);
                    if (decoded) {
                        assert(decoded == samples[letter].letter);
                        ++reads;
                    }
                    unsigned interval = now / 10u % 23u == 7u ? 20u : 10u;
                    position += interval * (speed + now / 180u % 3u * 3u - 3u);
                    now += interval;
                }
                if (reads != 1u) {
                    fprintf(stderr, "narrow=%u um spot=%u um letter=%c speed=%u mm/s reverse=%u reads=%u\n",
                            narrow_um, half_spot * 2u, samples[letter].letter, speed, reverse, reads);
                }
                assert(reads == 1u);
                assert(barcode_progress().bars - initial_bars == 15u);
            }
        }
    }
}

static void replay(const char *elements, char expected, bool reverse, unsigned speed,
                   unsigned phase, bool dropout, int paper, int bar)
{
    static const unsigned cadence[] = {10, 10, 20, 10, 30, 10, 10, 10};
    unsigned bounds[30] = {20000u};
    for (unsigned i = 0; i < 29; ++i) {
        unsigned width = elements[reverse ? 28 - i : i] == 'w' ? 15479u : 5160u;
        bounds[i + 1] = bounds[i] + width;
    }
    unsigned element = 0;
    unsigned distance = 0;
    unsigned reads = 0;
    unsigned tick = 0;
    barcode_reset();
    for (uint32_t now = phase; distance <= bounds[29] + 18000u; ) {
        while (element < 30 && distance >= bounds[element]) ++element;
        bool dark = element % 2 != 0;
        /* Model gradual transitions across the sensor's optical footprint. */
        const int edge_um = 1100;
        int shade = dark ? 256 : 0;
        if (element > 0 && distance - bounds[element - 1] < (unsigned)edge_um) {
            int blend = (int)(distance - bounds[element - 1]) * 128 / edge_um;
            shade = dark ? 128 + blend : 128 - blend;
        } else if (element < 30 && bounds[element] - distance < (unsigned)edge_um) {
            int blend = (int)(bounds[element] - distance) * 128 / edge_um;
            shade = dark ? 128 + blend : 128 - blend;
        }
        /* One bad sample inside a wide run. */
        if (tick % 41 == 17 && element > 0 && element < 30 &&
            distance > bounds[element - 1] + 2500u && distance + 2500u < bounds[element]) {
            shade = 256 - shade;
        }
        int adc = paper + (bar - paper) * shade / 256 + (int)(tick % 7) * 6 - 18;
        /* A single missed ADC read with a 20 ms gap should not erase the scan. */
        if (tick == 95) adc = -1;
        char decoded = barcode_feed(adc, now);
        if (decoded) { assert(decoded == expected && !dropout); ++reads; }
        unsigned interval = cadence[tick % 8];
        if (dropout && tick == 230) interval += 45;
        /* Modest speed ripple from the two-wheel correction. */
        distance += interval * (speed + tick / 12 % 3 * 4 - 4u);
        now += interval;
        ++tick;
    }
    assert(reads == (expected && !dropout ? 1u : 0u));
}

static void check_f_width_distortion(bool reverse, bool ambiguous)
{
    static const unsigned widths[][9] = {
        {130, 110, 320, 140, 300, 390, 160, 110, 110},
        {120, 120, 360, 120, 190, 360, 270, 120, 120},
    };
    unsigned ends[BARCODE_FRAME_ELEMENTS + 1u] = {1400u};
    for (unsigned i = 0; i < BARCODE_FRAME_ELEMENTS; ++i) {
        unsigned index = reverse ? BARCODE_FRAME_ELEMENTS - 1u - i : i;
        unsigned width = samples[1].elements[index] == 'w' ? 360u : 120u;
        if (index >= 10u && index < 19u) width = widths[ambiguous][index - 10u];
        ends[i + 1u] = ends[i] + width;
    }

    char letter = 0;
    unsigned misses = 0;
    unsigned element = 0;
    barcode_reset();
    uint32_t rejected_before = barcode_progress().width_rejects;
    for (uint32_t now = 0; now < ends[BARCODE_FRAME_ELEMENTS] + 200u; now += 10u) {
        while (element <= BARCODE_FRAME_ELEMENTS && now >= ends[element]) ++element;
        char decoded = barcode_feed(element % 2u ? 3000 : 220, now);
        if (barcode_progress().scan == BARCODE_SCAN_MISSED) ++misses;
        if (decoded) {
            assert(!letter);
            letter = decoded;
        }
    }
    if (ambiguous && letter) fprintf(stderr, "Distorted F decoded as %c\n", letter);
    assert(letter == (ambiguous ? 0 : 'F'));
    assert(barcode_progress().width_rejects - rejected_before == (ambiguous ? 1u : 0u));
    assert(misses == (ambiguous ? 1u : 0u));
}

static void check_incomplete_scan(unsigned elements, bool reverse)
{
    uint32_t edge = 1500u;
    unsigned index = 0;
    unsigned misses = 0;
    barcode_reset();
    for (uint32_t now = 0; now < 5400u; now += 10u) {
        if (index < elements && now == edge) {
            char width = samples[3].elements[reverse ? BARCODE_FRAME_ELEMENTS - 1u - index : index];
            edge += width == 'w' ? 360u : 120u;
            ++index;
        }
        bool dark = index % 2u && now < edge;
        assert(!barcode_feed(dark ? 3000 : 220, now));
        if (barcode_progress().scan == BARCODE_SCAN_MISSED) ++misses;
    }
    assert(misses == (elements >= 11u ? 1u : 0u));
}

int main(void)
{
    /* Image widths 338.67, 250.00 and 170.00 mm respectively. */
    check_print_size(6990u, 3250u);
    check_print_size(5160u, 3250u);
    check_print_size(3509u, 3250u);
    check_print_size(5160u, 4750u);
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        for (unsigned direction = 0; direction < 2; ++direction) {
            for (unsigned speed = 30; speed <= 58; speed += 7) {
                replay(samples[i].elements, samples[i].letter, direction, speed, 0, false, 220, 3000);
                replay(samples[i].elements, samples[i].letter, direction, speed, UINT32_MAX - 100,
                       false, 650, 980);  /* Black never crossed the old fixed threshold. */
                replay(samples[i].elements, samples[i].letter, direction, speed, 0,
                       false, 3380, 3920); /* Both colours clipped at 1000 before decoding. */
                replay(samples[i].elements, samples[i].letter, direction, speed, 0, false, 2500, 1920);
            }
        }
        replay(samples[i].elements, samples[i].letter, false, 42, 0, true, 130, 3200);
    }
    replay(samples[0].elements, 0, false, 42, 0, false, 800, 860); /* Insufficient contrast. */
    replay("nnnnnnnnnnnnnnnnnnnnnnnnnnnnn", 0, false, 42, 0, false, 130, 3200);
    for (unsigned direction = 0; direction < 2; ++direction) {
        check_f_width_distortion(direction, false);
        check_f_width_distortion(direction, true);
        check_incomplete_scan(1u, direction); /* A lone tape stripe. */
        check_incomplete_scan(9u, direction); /* A marker without payload. */
        check_incomplete_scan(13u, direction);
    }
    puts("PASS all three sizes, blurred narrow gaps, variable speed and contrast, and ambiguous F/I rejection");
}
