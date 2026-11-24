#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "s5_profile.h"
#include "s5_echo.h"

#ifdef CPU_RP2040
#include "../subsystem2/motion_hw.h"
#include <bsp/libbsp.h>
#include <stdint.h>

#define TRIG_PIN 16u
#define ECHO_PIN 17u
#define REG32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define PWM_SERVO (PWM_BASE + ((S5_SERVO_PIN >> 1u) & 7u) * 0x14u)
#define ECHO_BANK_OFFSET ((ECHO_PIN / 8u) * 4u)
#define ECHO_FALL (4u << ((ECHO_PIN % 8u) * 4u))
#define ECHO_RISE (8u << ((ECHO_PIN % 8u) * 4u))
#define ECHO_EDGES (ECHO_FALL | ECHO_RISE)
#define ECHO_INTR REG32(IO_BANK0_BASE + 0xf0u + ECHO_BANK_OFFSET)
#define ECHO_ENABLE REG32(echo_irq_base)
#define ECHO_PENDING REG32(echo_irq_base + 0x20u)
#define TRIGGER_US 12u
#define MICROS REG32(TIMER_TIMERAWL)
#define S5_TIMER_IRQ 0u
#define S5_ALARM_BIT 1u
#define PING_DONE 1u
#define PING_GUARD_MARGIN_MS 15
#define RESET_POLLS 32768u

static uintptr_t echo_irq_base;
static ID ping_event;
static bool pwm_ready;
static volatile S5_ECHO_CAPTURE capture;
static volatile unsigned observation_us;

_Static_assert(S5_ECHO_EXTENDED_US <= (1u << 17), "Echo pulse must fit in 17 bits");

static bool echo_high(void)
{
    return (REG32(GPIO_CTRL(ECHO_PIN) - 4u) & (1u << 17)) != 0;
}

static void finish_ping(void)
{
    ECHO_ENABLE &= ~ECHO_EDGES;
    REG32(TIMER_ARMED) = S5_ALARM_BIT;
    REG32(TIMER_INTR) = S5_ALARM_BIT;
    (void)tk_set_flg(ping_event, PING_DONE);
}

static void ping_alarm_irq(UINT irq)
{
    (void)irq;
    if (!(REG32(TIMER_INTS) & S5_ALARM_BIT)) return;
    REG32(TIMER_INTR) = S5_ALARM_BIT;
    if (capture.state != S5_CAPTURE_TRIGGER) {
        finish_ping();
        return;
    }

    ECHO_INTR = ECHO_EDGES;
    if (echo_high()) {
        capture.state = S5_CAPTURE_BAD;
        gpio_set_val(TRIG_PIN, 0);
        finish_ping();
        return;
    }
    capture.triggered_at = MICROS;
    capture.state = S5_CAPTURE_RISE;
    REG32(TIMER_ALARM0) = capture.triggered_at + S5_ECHO_TIMEOUT_US;
    ECHO_ENABLE |= ECHO_EDGES;
    gpio_set_val(TRIG_PIN, 0);
}

void s5_hw_echo_irq(void)
{
    if (!echo_irq_base) return;
    uint32_t edges = ECHO_PENDING & ECHO_EDGES;
    if (!edges) return;
    uint32_t now = MICROS;
    ECHO_INTR = edges;
    unsigned previous = capture.state;
    s5_capture_edge(&capture, now, (edges & ECHO_RISE) != 0, (edges & ECHO_FALL) != 0, observation_us);
    if (previous == S5_CAPTURE_RISE && capture.state == S5_CAPTURE_FALL) {
        REG32(TIMER_ALARM0) = capture.triggered_at + observation_us;
        REG32(TIMER_INTR) = S5_ALARM_BIT;
    } else if (capture.state == S5_CAPTURE_DONE || capture.state == S5_CAPTURE_LATE ||
               capture.state == S5_CAPTURE_BAD) {
        finish_ping();
    }
}

static bool release_reset(unsigned mask)
{
    REG32(RESETS_RESET + 0x3000u) = mask;
    for (unsigned i = 0; i < RESET_POLLS; ++i) {
        if ((REG32(RESETS_RESET_DONE) & mask) == mask) return true;
    }
    return false;
}

bool s5_hw_init_ranging(void)
{
    gpio_set_val(TRIG_PIN, 0);
    gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);
    /* A connected push-pull output drives idle low. An unplugged input reads high. */
    REG32(GPIO(ECHO_PIN)) = (REG32(GPIO(ECHO_PIN)) & ~GPIO_PDE) | GPIO_PUE;

#if APP_S5_BENCH
    app_rp2040_timebase_init();
    if (app_gpio_irq_init() < E_OK) return false;
#endif
    T_CFLG event = {0};
    ID id = tk_cre_flg(&event);
    if (id < E_OK) return false;
    T_DINT alarm = {.intatr = TA_HLNG, .inthdr = (FP)ping_alarm_irq};
    if (tk_def_int(S5_TIMER_IRQ, &alarm) < E_OK) {
        (void)tk_del_flg(id);
        return false;
    }
    /* In production this task and the shared encoder ISR run on processor 2. */
    unsigned cpu = 0;
#if TK_SUPPORT_SMP
    cpu = (unsigned)tk_get_prc() - 1u;
#endif
    UW saved;
    DI(saved);
    echo_irq_base = IO_BANK0_BASE + 0x100u + cpu * 0x30u + ECHO_BANK_OFFSET;
    ECHO_ENABLE &= ~ECHO_EDGES;
    ECHO_INTR = ECHO_EDGES;
    ping_event = id;
    REG32(TIMER_ARMED) = S5_ALARM_BIT;
    REG32(TIMER_INTR) = S5_ALARM_BIT;
    REG32(TIMER_INTE + 0x2000u) = S5_ALARM_BIT;
    ClearInt(S5_TIMER_IRQ);
    EnableInt(S5_TIMER_IRQ, 3); /* Same priority as the shared GPIO IRQ. */
    EI(saved);
    return true;
}

bool s5_hw_init(void)
{
    pwm_ready = release_reset(RESETS_RESET_PWM);
    if (!pwm_ready) return false;

    pwm_set_enabled(S5_SERVO_PIN, FALSE);
    REG32(PWM_SERVO) = 0;
    REG32(PWM_SERVO + 4u) = 125u * 16u; /* 1 us/count at 125 MHz. */
    REG32(PWM_SERVO + 8u) = 0;
    pwm_set_wrap(S5_SERVO_PIN, 19999u); /* 50 Hz. */
    s5_hw_angle(S5_CENTRE_ANGLE);
    pwm_set_pin(S5_SERVO_PIN);
    pwm_set_enabled(S5_SERVO_PIN, TRUE);
    return s5_hw_init_ranging();
}

static unsigned servo_calibrated_pulse(unsigned angle)
{
    _Static_assert(S5_SERVO_RIGHT_US < S5_SERVO_CENTRE_US &&
                   S5_SERVO_CENTRE_US < S5_SERVO_LEFT_US &&
                   S5_SERVO_RIGHT_US > 0u && S5_SERVO_LEFT_US < 20000u,
                   "Servo pulses must increase from right through centre to left within one PWM period");
#ifdef S5_SERVO_RIGHT_OUTER_US
    _Static_assert(S5_SERVO_RIGHT_OUTER_US > 0u && S5_SERVO_RIGHT_OUTER_US < S5_SERVO_RIGHT_US,
                   "The calibrated 15-degree pulse must precede the 30-degree pulse");
    if (angle <= S5_SCAN_MIN_ANGLE) return S5_SERVO_RIGHT_OUTER_US;
    if (angle < S5_MIN_ANGLE) {
        const unsigned travel = S5_MIN_ANGLE - S5_SCAN_MIN_ANGLE;
        return S5_SERVO_RIGHT_OUTER_US +
               ((angle - S5_SCAN_MIN_ANGLE) * (S5_SERVO_RIGHT_US - S5_SERVO_RIGHT_OUTER_US) +
                travel / 2u) / travel;
    }
#endif
#ifdef S5_SERVO_LEFT_OUTER_US
    _Static_assert(S5_SERVO_LEFT_OUTER_US > S5_SERVO_LEFT_US && S5_SERVO_LEFT_OUTER_US < 20000u,
                   "The calibrated 165-degree pulse must follow the 150-degree pulse within one PWM period");
    if (angle >= S5_SCAN_MAX_ANGLE) return S5_SERVO_LEFT_OUTER_US;
    if (angle > S5_MAX_ANGLE) {
        const unsigned travel = S5_SCAN_MAX_ANGLE - S5_MAX_ANGLE;
        return S5_SERVO_LEFT_US +
               ((angle - S5_MAX_ANGLE) * (S5_SERVO_LEFT_OUTER_US - S5_SERVO_LEFT_US) +
                travel / 2u) / travel;
    }
#endif
    if (angle <= S5_MIN_ANGLE) return S5_SERVO_RIGHT_US;
    if (angle >= S5_MAX_ANGLE) return S5_SERVO_LEFT_US;

    const unsigned half_sweep = S5_MAX_ANGLE - S5_CENTRE_ANGLE;
    if (angle <= S5_CENTRE_ANGLE)
        return S5_SERVO_CENTRE_US -
               ((S5_CENTRE_ANGLE - angle) * (S5_SERVO_CENTRE_US - S5_SERVO_RIGHT_US) +
                half_sweep / 2u) / half_sweep;
    return S5_SERVO_CENTRE_US +
           ((angle - S5_CENTRE_ANGLE) * (S5_SERVO_LEFT_US - S5_SERVO_CENTRE_US) +
            half_sweep / 2u) / half_sweep;
}

unsigned s5_servo_pulse_us(unsigned angle)
{
    unsigned pulse=servo_calibrated_pulse(angle);
#if S5_SERVO_REVERSED
    return 2u*S5_SERVO_CENTRE_US-pulse;
#else
    return pulse;
#endif
}

void s5_hw_angle(unsigned angle)
{
    if (pwm_ready)
        pwm_set_cc(S5_SERVO_PIN, s5_servo_pulse_us(angle));
}

static S5_PING ping(unsigned observe_us)
{
    if (!ping_event) return s5_classify_echo(S5_ECHO_TIMING, 0);
    if (echo_high()) return s5_classify_echo(S5_ECHO_HIGH, 0);

    (void)tk_clr_flg(ping_event, ~PING_DONE);
    UW saved;
    DI(saved);
    observation_us = observe_us;
    capture.state = S5_CAPTURE_TRIGGER;
    gpio_set_val(TRIG_PIN, 1);
    REG32(TIMER_ALARM0) = MICROS + TRIGGER_US;
    EI(saved);

    /* The kernel timeout also bounds failure of the hardware timer or IRQ. */
    UINT events;
    TMO guard_ms = (TMO)(observe_us / 1000u) + PING_GUARD_MARGIN_MS;
    ER result = tk_wai_flg(ping_event, PING_DONE, TWF_ORW | TWF_BITCLR, &events, guard_ms);
    DI(saved);
    gpio_set_val(TRIG_PIN, 0);
    REG32(TIMER_ARMED) = S5_ALARM_BIT;
    REG32(TIMER_INTR) = S5_ALARM_BIT;
    ECHO_ENABLE &= ~ECHO_EDGES;
    ECHO_INTR = ECHO_EDGES;
    const S5_ECHO_CAPTURE sample = capture;
    uint32_t finished_at = MICROS;
    capture.state = S5_CAPTURE_IDLE;
    EI(saved);
    return s5_capture_result(&sample, finished_at, result == E_OK, false);
}

S5_PING s5_hw_ping(void)
{
    return ping(S5_ECHO_OBSERVE_US);
}

S5_PING s5_hw_ping_extended(void)
{
    return ping(S5_ECHO_EXTENDED_US);
}
#endif
