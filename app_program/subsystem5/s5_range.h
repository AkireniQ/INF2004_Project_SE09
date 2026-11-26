#ifndef S5_RANGE_H
#define S5_RANGE_H

#include <stdbool.h>

#define S5_MAX_RANGE_MM 4000u
#define S5_MIN_ECHO_US 100u
#define S5_ECHO_TIMEOUT_US 30000u
#define S5_ECHO_OBSERVE_US 60000u
#define S5_ECHO_EXTENDED_US 100000u
/* RCWL-9610A no-target pulse observed at approximately 47.1 ms on the car. */
#define S5_NO_TARGET_MIN_US 46900u
#define S5_NO_TARGET_MAX_US 47300u
#define S5_OPEN_RANGE 4094u
#define S5_UNKNOWN_RANGE 4095u
#define S5_SCAN_STOP_MM 240u
#define S5_CLEAR_MM 270u
#define S5_FOLLOW_STOP_MM 170u
#define S5_FOLLOW_CLEAR_MM 270u

static inline bool s5_range_measured(unsigned range)
{
    return range <= S5_MAX_RANGE_MM;
}

static inline bool s5_range_available(unsigned range)
{
    return s5_range_measured(range) || range == S5_OPEN_RANGE;
}

static inline unsigned s5_range_from_echo(unsigned pulse_us)
{
    if (pulse_us < S5_MIN_ECHO_US || pulse_us >= S5_ECHO_EXTENDED_US)
        return S5_UNKNOWN_RANGE;
    unsigned mm = (pulse_us * 343u + 1000u) / 2000u;
    return s5_range_measured(mm) ? mm : S5_OPEN_RANGE;
}

typedef enum {
    S5_ECHO_QUIET, S5_ECHO_PULSE, S5_ECHO_HIGH, S5_ECHO_TIMING, S5_ECHO_FALL_TIMEOUT,
    S5_ECHO_LATE, S5_ECHO_NO_TARGET
} S5_ECHO_STATE;

typedef struct {
    unsigned pulse_us:17;
    unsigned range_mm:12;
    unsigned state:3;
} S5_PING;

static inline S5_PING s5_classify_echo(S5_ECHO_STATE state, unsigned pulse_us)
{
    unsigned range = S5_UNKNOWN_RANGE;
    if (state == S5_ECHO_PULSE || state == S5_ECHO_LATE)
        range = s5_range_from_echo(pulse_us);
    if (state == S5_ECHO_LATE &&
        pulse_us >= S5_NO_TARGET_MIN_US && pulse_us <= S5_NO_TARGET_MAX_US) {
        state = S5_ECHO_NO_TARGET;
    }
    return (S5_PING){.state = state, .pulse_us = pulse_us, .range_mm = range};
}

/* Require agreement at a fine-scan bearing and keep the shorter measured range. */
#define S5_FINE_REPEAT_TOLERANCE_MM 25u
static inline unsigned s5_confirm_scan_range(unsigned previous, unsigned current)
{
    if (previous == S5_OPEN_RANGE && current == S5_OPEN_RANGE) return S5_OPEN_RANGE;
    if (s5_range_measured(previous) && s5_range_measured(current) &&
        previous <= current + S5_FINE_REPEAT_TOLERANCE_MM &&
        current <= previous + S5_FINE_REPEAT_TOLERANCE_MM)
        return previous < current ? previous : current;
    return S5_UNKNOWN_RANGE;
}

typedef struct {
    unsigned range_mm:12;
    unsigned first_open:1;
    unsigned blocked:1;
} S5_RANGE_GATE;

/* Confirm open space across two pings before clearing a close return or fault.
 * The caller refreshes the accepted sample's timestamp only on success. */
static inline bool s5_accept_range(S5_RANGE_GATE *gate, unsigned range)
{
    bool open = range == S5_OPEN_RANGE;
    bool confirm = !open || gate->first_open;
    gate->first_open = open;
    if (!confirm) return false;
    gate->range_mm = s5_range_available(range) ? range : S5_UNKNOWN_RANGE;
    gate->blocked = !s5_range_available(range) ||
                    range <= (gate->blocked ? S5_CLEAR_MM : S5_SCAN_STOP_MM);
    return true;
}

#endif
