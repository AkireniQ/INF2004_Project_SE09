#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

/* Letter indices 1..26 represent A..Z. */
typedef struct {
    unsigned left:5, right:5, straight:5, uturn:5;
} BARCODE_MAP;

static inline bool barcode_map_valid(BARCODE_MAP map)
{
    unsigned letters[] = {map.left, map.right, map.straight, map.uturn};
    uint32_t seen = 0;
    for (unsigned i = 0; i < 4; ++i) {
        unsigned letter = letters[i];
        if (!letter || letter > 26 || (seen & (1u << letter))) return false;
        seen |= 1u << letter;
    }
    return true;
}

void barcode_reset(void);
/* Feed a timed GP27 ADC sample (0..4095, or -1 when unavailable). */
char barcode_feed(int adc, uint32_t now_ms);
#define BARCODE_FRAME_ELEMENTS 29u
#define BARCODE_MAX_SAMPLE_GAP_MS 35u
typedef struct {
    uint32_t bars; /* Tentative total, adjusted if the contrast is recalibrated. */
    uint32_t width_rejects;
    uint8_t elements;
} BARCODE_PROGRESS;
/* Read only from the task that feeds the decoder. */
BARCODE_PROGRESS barcode_progress(void);

#endif
