#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "../../app_program/subsystem3/barcode.h"

/* BARCODE (1).pptx, slides 3, 7, 11. Medium image placed at 250 mm. */
static const struct {
    char letter;
    const char *elements;
} samples[] = {
    {'A', "nwnnwnwnn" "n" "wnnnnwnnw" "n" "nwnnwnwnn"},
    {'F', "nwnnwnwnn" "n" "nnwnwwnnn" "n" "nwnnwnwnn"},
    {'Z', "nwnnwnwnn" "n" "nwwnwnnnn" "n" "nwnnwnwnn"},
};

static void replay(const char *elements, char expected, bool reverse, unsigned speed,
                   unsigned phase, bool dropout)
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
        /* One bad sample inside a wide run. */
        if (tick % 41 == 17 && element > 0 && element < 30 &&
            distance > bounds[element - 1] + 2500u && distance + 2500u < bounds[element]) {
            dark = !dark;
        }
        char decoded = barcode_feed(dark ? 900 : 60, now);
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

int main(void)
{
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        for (unsigned direction = 0; direction < 2; ++direction) {
            for (unsigned speed = 30; speed <= 58; speed += 7) {
                replay(samples[i].elements, samples[i].letter, direction, speed, 0, false);
                replay(samples[i].elements, samples[i].letter, direction, speed, UINT32_MAX - 100, false);
            }
        }
        replay(samples[i].elements, samples[i].letter, false, 42, 0, true);
    }
    replay("nnnnnnnnnnnnnnnnnnnnnnnnnnnnn", 0, false, 42, 0, false);
    puts("PASS medium A/F/Z in both directions, irregular ticks, isolated noise and frame loss");
}
