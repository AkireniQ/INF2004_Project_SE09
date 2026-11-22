#ifndef S5_ECHO_H
#define S5_ECHO_H

#include "s5_range.h"
#include <stdint.h>

enum { S5_CAPTURE_IDLE, S5_CAPTURE_RISE, S5_CAPTURE_FALL, S5_CAPTURE_DONE,
       S5_CAPTURE_BAD, S5_CAPTURE_LATE, S5_CAPTURE_TRIGGER };

typedef struct {
    uint32_t triggered_at, rose_at;
    unsigned state : 3;
    unsigned pulse_us : 16;
} S5_ECHO_CAPTURE;

/* The task arms and drains this state with the local interrupt masked. */
static inline void s5_capture_edge(volatile S5_ECHO_CAPTURE *capture, uint32_t now,
                                   bool rise, bool fall, uint32_t observe_us)
{
    uint32_t elapsed = now - capture->triggered_at;
    bool late = elapsed >= S5_ECHO_TIMEOUT_US;
    /* The extended window only observes the end of a pulse already in progress. */
    if (elapsed >= observe_us || (late && capture->state != S5_CAPTURE_FALL)) return;
    if (capture->state == S5_CAPTURE_RISE) {
        if (rise && !fall) {
            capture->rose_at = now;
            capture->state = S5_CAPTURE_FALL;
        } else {
            capture->state = S5_CAPTURE_BAD;
        }
    } else if (capture->state == S5_CAPTURE_FALL) {
        if (fall && !rise) {
            capture->pulse_us = now - capture->rose_at;
            capture->state = late ? S5_CAPTURE_LATE : S5_CAPTURE_DONE;
        } else {
            capture->state = S5_CAPTURE_BAD;
        }
    }
}

static inline S5_PING s5_capture_result(const S5_ECHO_CAPTURE *capture, uint32_t now,
                                       bool triggered, bool idle_high)
{
    if (triggered) {
        if (idle_high) return s5_classify_echo(S5_ECHO_HIGH, 0);
        if (capture->state == S5_CAPTURE_DONE)
            return s5_classify_echo(S5_ECHO_PULSE, capture->pulse_us);
        if (capture->state == S5_CAPTURE_LATE)
            return s5_classify_echo(S5_ECHO_LATE, capture->pulse_us);
        if (now - capture->triggered_at >= S5_ECHO_TIMEOUT_US) {
            if (capture->state == S5_CAPTURE_RISE) return s5_classify_echo(S5_ECHO_QUIET, 0);
            if (capture->state == S5_CAPTURE_FALL) return s5_classify_echo(S5_ECHO_FALL_TIMEOUT, 0);
        }
    }
    return s5_classify_echo(S5_ECHO_TIMING, 0);
}

#endif
