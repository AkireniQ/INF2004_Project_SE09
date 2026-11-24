#ifndef MOTION_CRAWL_H
#define MOTION_CRAWL_H

#include <stdbool.h>
#include <stdint.h>

#define S2_CRAWL_MAX_RPM 16u
#define S2_CRAWL_WINDOW 8u /* Eight 10 ms encoder releases. */
#define S2_RPM_SCALE 256u

typedef struct {
    uint32_t window_us;
    uint16_t elapsed_us[S2_CRAWL_WINDOW];
    uint16_t pulses[S2_CRAWL_WINDOW];
    uint16_t total_pulses[2];
    int32_t integral[2];
    uint8_t assist_ticks[2];
    uint8_t next;
} S2_CRAWL;

/* The motor task supplies the same atomic count sample used by legacy motion. */
void s2_crawl_sample(S2_CRAWL *crawl, uint16_t elapsed_us, uint16_t sample,
                     uint32_t rpm[2]);

/* Returns PWM compare counts on the existing 0..1000 scale. */
unsigned s2_crawl_duty(S2_CRAWL *crawl, unsigned wheel, unsigned target,
                       const uint32_t rpm[2], bool match_wheels, bool allow_assist);

#endif
