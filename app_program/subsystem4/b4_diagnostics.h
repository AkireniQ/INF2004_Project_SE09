/* Test-build diagnostics policy, independent of acquisition and safety. */
#ifndef B4_DIAGNOSTICS_H
#define B4_DIAGNOSTICS_H
#include <stdint.h>
#include "b4_event.h"
#include "b4_lsm303dlhc.h"

typedef struct { uint8_t muted; } B4_DIAGNOSTICS;
static inline void b4_diagnostics_stop(B4_DIAGNOSTICS *state)
{ state->muted = 1U; }
static inline void b4_diagnostics_rearm(B4_DIAGNOSTICS *state, uint8_t accepted)
{ if (accepted) state->muted = 0U; }
static inline uint8_t b4_diagnostics_error(int32_t read_status, int32_t io_error,
                                           int32_t encode_status, uint8_t events_dropped)
{
    return (read_status != B4_LSM303DLHC_STATUS_OK &&
            read_status != B4_LSM303DLHC_STATUS_NO_FRESH_DATA) ||
           io_error < 0 || encode_status != B4_EVENT_STATUS_OK || events_dropped != 0U;
}
#endif
