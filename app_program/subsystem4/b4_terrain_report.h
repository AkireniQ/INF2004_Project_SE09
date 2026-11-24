#ifndef B4_TERRAIN_REPORT_H
#define B4_TERRAIN_REPORT_H
#include <stdint.h>
#include "b4_processor.h"

/* A bounded FIFO holds completed valid heights. The maximum is extracted
 * separately from the processor so it survives FIFO overflow. */
#define B4_TERRAIN_QUEUE_SIZE 8U
#define B4_TERRAIN_HOLD_MS 1000U
#define B4_TERRAIN_SAMPLE_MAX_AGE_MS 1000U
#define B4_TERRAIN_PHASE_UNAVAILABLE 3U
#define B4_TERRAIN_HEIGHT_MAX_MM 2000U
#define B4_TERRAIN_HEIGHT_UNAVAILABLE 2047U

typedef struct {
    uint16_t heights[B4_TERRAIN_QUEUE_SIZE];
    uint8_t head, count, phase, have_height, holding, sample_valid, offered;
    uint16_t displayed_height, max_height_mm;
    uint32_t hold_since, sample_at, dropped;
} B4_TERRAIN_REPORT;

void b4_terrain_report_observe(B4_TERRAIN_REPORT *report,
                               const B4_PROCESSOR_OUTPUT *output,
                               uint32_t timestamp_ms);
/* Called periodically. Offline time never consumes the FIFO. */
uint8_t b4_terrain_report_next(B4_TERRAIN_REPORT *report, uint32_t now_ms,
                               uint8_t connected, uint16_t *height,
                               uint16_t *max_height, uint8_t *phase);
/* Commit only after the controller accepts the offered measurement. */
void b4_terrain_report_commit(B4_TERRAIN_REPORT *report, uint32_t now_ms);
#endif
