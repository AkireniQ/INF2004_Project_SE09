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
/* Feed a timed darkness sample. Returns a letter once per complete frame. */
char barcode_feed(int darkness, uint32_t now_ms);

#endif
