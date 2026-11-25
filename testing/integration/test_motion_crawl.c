#include <assert.h>
#include <stdio.h>

/* Run the firmware motion core with a deterministic clock, encoder and PWM. */
#define APP_H
#define EXPORT
#define E_OK 0
#define DI(saved) ((saved) = 0)
#define EI(saved) ((void)(saved))
typedef int ER;
typedef int INT;
typedef unsigned UW;
#include "../../app_program/subsystem2/motion_core.c"

static uint64_t clock_us;
static unsigned pwm[32];

uint64_t s2_hw_time_us(void) { return clock_us; }
void s2_hw_pwm_init(void) {}
ER s2_hw_encoder_init(void) { return E_OK; }
void s2_hw_pwm_write(unsigned pin, unsigned level) { pwm[pin] = level; }

static void tick(unsigned left, unsigned right)
{
    for (unsigned i = 0; i < left; ++i) {
        subsystem2_encoder_gpio_irq(LEFT_ENC_A, S2_GPIO_EDGE_RISE);
    }
    for (unsigned i = 0; i < right; ++i) {
        subsystem2_encoder_gpio_irq(RIGHT_ENC_A, S2_GPIO_EDGE_RISE);
    }
    clock_us += 10000u;
    s2_core_service();
}

static unsigned measured(unsigned wheel)
{
    S2_STATUS snapshot;
    s2_core_snapshot(&snapshot);
    return s2_get_bits(&snapshot, wheel == 0u ? 9u : 18u, 9u);
}

static unsigned absolute_difference(unsigned left, unsigned right)
{
    return left > right ? left - right : right - left;
}

static void check_window(void)
{
    s2_core_init();
    assert(s2_core_set_wheel_rpm(10, 10));
    for (unsigned i = 0; i < 32u; ++i) {
        /* Identical speed with the encoders half a sample out of phase. */
        tick(i % 2u == 0u, i % 2u != 0u);
        if (i >= 7u) {
            assert(measured(0) == 5u && measured(1) == 5u);
            assert(absolute_difference(pwm[LEFT_IN_A], pwm[RIGHT_IN_A]) <= 8u);
        }
    }
    for (unsigned i = 0; i < S2_CRAWL_WINDOW; ++i) { tick(0, 0); }
    assert(measured(0) == 0u && measured(1) == 0u);
}

static void check_coupling(void)
{
    S2_CRAWL straight = {0};
    S2_CRAWL curve = {0};
    uint32_t rpm[2] = {11u * S2_RPM_SCALE, 2u * S2_RPM_SCALE};
    unsigned fast = s2_crawl_duty(&straight, 0, 10, rpm, true, false);
    unsigned slow = s2_crawl_duty(&straight, 1, 10, rpm, true, false);
    unsigned independent_fast = s2_crawl_duty(&curve, 0, 10, rpm, false, false);
    unsigned independent_slow = s2_crawl_duty(&curve, 1, 10, rpm, false, false);
    assert(fast < independent_fast && slow > independent_slow);
}

static void check_drivetrain(void)
{
    const unsigned motor_pin[2] = {LEFT_IN_A, RIGHT_IN_A};
    const double drag_duty[2] = {48.0, 92.0};
    const double teeth[2] = {LEFT_CPR_X10 / 60000.0, RIGHT_CPR_X10 / 60000.0};
    double encoder_phase[2] = {0.35, 0.85};
    double speed[2] = {0.0, 0.0};
    unsigned worst_gap = 0;
    double actual_gap = 0.0;

    s2_core_init();
    assert(s2_core_set_wheel_rpm(10, 10));
    for (unsigned i = 0; i < 650u; ++i) {
        unsigned pulses[2];
        for (unsigned wheel = 0; wheel < 2u; ++wheel) {
            double net_duty = pwm[motor_pin[wheel]] - drag_duty[wheel];
            double equilibrium_rpm = net_duty > 0.0 ? net_duty / 6.0 : 0.0;
            speed[wheel] += (equilibrium_rpm - speed[wheel]) / 7.0;
            encoder_phase[wheel] += speed[wheel] * teeth[wheel];
            pulses[wheel] = (unsigned)encoder_phase[wheel];
            encoder_phase[wheel] -= pulses[wheel];
        }
        tick(pulses[0], pulses[1]);
        assert(s2_core_faults() == 0u);
        if (i >= 350u) {
            unsigned left = measured(0), right = measured(1);
            assert(left >= 8u && left <= 12u && right >= 8u && right <= 12u);
            unsigned gap = absolute_difference(left, right);
            if (gap > worst_gap) { worst_gap = gap; }
            double speed_difference = speed[0] - speed[1];
            actual_gap += speed_difference < 0.0 ? -speed_difference : speed_difference;
        }
    }
    assert(worst_gap <= 2u && actual_gap / 300.0 < 1.0);

    /* A tight turn can still pause the inside wheel on the original path. */
    assert(s2_core_set_wheel_rpm(0, 12));
    tick(0, 1);
    assert(pwm[LEFT_IN_A] == 1000u && pwm[LEFT_IN_B] == 1000u);
    assert(pwm[RIGHT_IN_A] > 0u);
    s2_core_stop();
    assert(pwm[LEFT_IN_A] == 0u && pwm[LEFT_IN_B] == 0u);
    assert(pwm[RIGHT_IN_A] == 0u && pwm[RIGHT_IN_B] == 0u);

    assert(s2_core_turn_left(1));
    tick(0, 0);
    assert(pwm[LEFT_IN_A] == 0u && pwm[LEFT_IN_B] > 0u);
    assert(pwm[RIGHT_IN_A] > 0u && pwm[RIGHT_IN_B] == 0u);
}

static void check_hump_load(void)
{
    s2_core_init();
    assert(s2_core_set_wheel_rpm(12, 12));
    for (unsigned i = 0; i < 250u; ++i) {
        /* About 12 RPM on level ground. */
        unsigned pulses = i % 4u == 0u ? 2u : 1u;
        tick(pulses, pulses);
    }
    assert(crawl.assist_ticks[0] == 0 && crawl.assist_ticks[1] == 0);

    bool moving_over_hump = false;
    for (unsigned i = 0; i < 90u; ++i) {
        /* The loaded right wheel needs more PWM than the normal crawl cap. */
        moving_over_hump = pwm[RIGHT_IN_A] > 450u;
        tick(1, moving_over_hump ? 1u : 0u);
        assert(!s2_core_faults() && pwm[RIGHT_IN_A] <= 600u);
        if (moving_over_hump) break;
    }
    assert(moving_over_hump && crawl.assist_ticks[0] == 0 && crawl.assist_ticks[1] > 0);
    assert(state.left_rpm == 12 && state.right_rpm == 12);

    for (unsigned i = 0; i < 28u; ++i) tick(1, 1);
    assert(crawl.assist_ticks[1] == 0 && pwm[RIGHT_IN_A] <= 450u);
}

static void check_stall(void)
{
    s2_core_init();
    assert(s2_core_set_wheel_rpm(10, 10));
    for (unsigned i = 0; i < 220u && s2_core_faults() == 0u; ++i) { tick(0, 0); }
    assert((s2_core_faults() & FAULT_STALL) != 0u);
    assert(pwm[LEFT_IN_A] == 0u && pwm[RIGHT_IN_A] == 0u);
}

int main(void)
{
    check_window();
    check_coupling();
    check_drivetrain();
    check_hump_load();
    check_stall();
    puts("PASS low-speed encoder filtering, unequal wheel drag, turn output and stall protection");
    return 0;
}
