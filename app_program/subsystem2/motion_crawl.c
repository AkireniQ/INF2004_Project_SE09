#include "motion_crawl.h"
#include "motion_config.h"

#define CRAWL_PWM_MAX 450
#define CRAWL_INTEGRAL_MAX (400 * (int32_t)S2_RPM_SCALE)
#define CRAWL_BALANCE_LIMIT (40 * (int32_t)S2_RPM_SCALE)

static int32_t clamp(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) { return lower; }
    if (value > upper) { return upper; }
    return value;
}

void s2_crawl_sample(S2_CRAWL *crawl, uint16_t elapsed_us, uint16_t sample,
                     uint32_t rpm[2])
{
    const unsigned cpr_x10[2] = {LEFT_CPR_X10, RIGHT_CPR_X10};
    unsigned index = crawl->next;
    uint16_t previous = crawl->pulses[index];
    crawl->pulses[index] = sample;
    crawl->window_us += (uint32_t)elapsed_us;
    crawl->window_us -= crawl->elapsed_us[index];
    crawl->elapsed_us[index] = elapsed_us;
    crawl->next = (uint8_t)((index + 1u) % S2_CRAWL_WINDOW);

    for (unsigned wheel = 0; wheel < 2u; ++wheel) {
        unsigned shift = wheel * 5u;
        unsigned edges = (sample >> shift) & 31u;
        unsigned expired = (previous >> shift) & 31u;
        crawl->total_pulses[wheel] = (uint16_t)(crawl->total_pulses[wheel] + edges - expired);
        uint64_t scaled = (uint64_t)crawl->total_pulses[wheel] * 600000000u * S2_RPM_SCALE;
        uint64_t denominator = (uint64_t)cpr_x10[wheel] * crawl->window_us;
        uint32_t value = denominator ? (uint32_t)(scaled / denominator) : 0u;
        rpm[wheel] = value > 300u * S2_RPM_SCALE ? 300u * S2_RPM_SCALE : value;
    }
}

unsigned s2_crawl_duty(S2_CRAWL *crawl, unsigned wheel, unsigned target,
                       const uint32_t rpm[2], bool match_wheels)
{
    int32_t error = (int32_t)(target * S2_RPM_SCALE) - (int32_t)rpm[wheel];
    int32_t integral = clamp(crawl->integral[wheel] + error / 6,
                             -CRAWL_INTEGRAL_MAX, CRAWL_INTEGRAL_MAX);
    int32_t base = (int32_t)(target * 1000u * S2_RPM_SCALE / MAX_RPM);
    int32_t trim = 0;

    if (match_wheels) {
        int32_t difference = (int32_t)rpm[wheel ^ 1u] - (int32_t)rpm[wheel];
        /* A difference of about one RPM can come from encoder pulse phase. */
        const int32_t deadband = (int32_t)S2_RPM_SCALE;
        difference -= clamp(difference, -deadband, deadband);
        trim = clamp(3 * difference, -CRAWL_BALANCE_LIMIT, CRAWL_BALANCE_LIMIT);
    }

    const int32_t maximum = CRAWL_PWM_MAX * (int32_t)S2_RPM_SCALE;
    int32_t duty = base + 2 * error + integral + trim;
    if ((duty <= maximum || error < 0) && (duty >= 0 || error > 0)) {
        crawl->integral[wheel] = integral;
    }
    return (unsigned)((clamp(duty, 0, maximum) + S2_RPM_SCALE / 2u) / S2_RPM_SCALE);
}
