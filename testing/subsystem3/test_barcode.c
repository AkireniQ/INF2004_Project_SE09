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

int main(void)
{
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
    puts("PASS medium A/F/Z with weak, shifted and inverted signals; low contrast and broken frames rejected");
}
