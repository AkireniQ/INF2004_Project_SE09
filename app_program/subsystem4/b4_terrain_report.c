#include "b4_terrain_report.h"
#include <stddef.h>
#include <limits.h>

void b4_terrain_report_observe(B4_TERRAIN_REPORT *r,
                               const B4_PROCESSOR_OUTPUT *out, uint32_t at)
{
    if (!r || !out) return;
    if (out->roll_valid) {
        r->sample_valid = 1U;
        r->sample_at = at;
        r->phase = out->hump_state == B4_HUMP_RIGHT_RISE
            ? B4_TERRAIN_ASCENDING : out->terrain_state;
    }
    if (!out->hump_detected || !out->height_valid || out->peak_height_mm < 0 ||
        (uint32_t)out->peak_height_mm > B4_TERRAIN_HEIGHT_MAX_MM) return;
    r->max_height_mm = out->max_height_valid && out->max_height_mm >= out->peak_height_mm &&
                       (uint32_t)out->max_height_mm <= B4_TERRAIN_HEIGHT_MAX_MM
                       ? (uint16_t)out->max_height_mm : B4_TERRAIN_HEIGHT_UNAVAILABLE;
    if (r->count == B4_TERRAIN_QUEUE_SIZE) {
        if (r->dropped < UINT32_MAX) ++r->dropped;
        return;
    }
    unsigned tail = (r->head + r->count) % B4_TERRAIN_QUEUE_SIZE;
    r->heights[tail] = (uint16_t)out->peak_height_mm;
    ++r->count;
}

uint8_t b4_terrain_report_next(B4_TERRAIN_REPORT *r, uint32_t now,
                               uint8_t connected, uint16_t *height,
                               uint16_t *max_height, uint8_t *phase)
{
    if (!r || !height || !max_height || !phase || !connected) {
        if (r && !connected) r->holding = 0U;
        return 0U;
    }
    r->offered = 0U;
    if (r->holding && (uint32_t)(now - r->hold_since) >= B4_TERRAIN_HOLD_MS)
        r->holding = 0U;
    bool live = r->sample_valid && (uint32_t)(now - r->sample_at) <= B4_TERRAIN_SAMPLE_MAX_AGE_MS;
    if (r->count && !r->holding) {
        *height = r->heights[r->head];
        r->offered = 1U;
    } else if (r->holding) {
        *height = r->displayed_height;
    } else if (live) {
        *height = 0U;
    } else return 0U;
    *max_height = r->have_height || r->count ? r->max_height_mm : B4_TERRAIN_HEIGHT_UNAVAILABLE;
    *phase = live ? r->phase : B4_TERRAIN_PHASE_UNAVAILABLE;
    return 1U;
}

void b4_terrain_report_commit(B4_TERRAIN_REPORT *r, uint32_t now)
{
    if (!r || !r->offered || !r->count) return;
    r->displayed_height = r->heights[r->head];
    r->head = (uint8_t)((r->head + 1U) % B4_TERRAIN_QUEUE_SIZE);
    --r->count;
    r->have_height = r->holding = 1U;
    r->hold_since = now;
    r->offered = 0U;
}
