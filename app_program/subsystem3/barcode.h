#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>

void barcode_reset(void);
/* Feed a timed darkness sample. Returns a letter once per complete frame. */
char barcode_feed(int darkness, uint32_t now_ms);

#endif
