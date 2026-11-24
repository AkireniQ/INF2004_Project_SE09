/* cc -std=c11 -Wall -Wextra -Werror -pedantic testing/buddy4/test_b4_diagnostics.c -o /tmp/test_b4_diagnostics && /tmp/test_b4_diagnostics */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem4/b4_diagnostics.h"

int main(void)
{
    B4_DIAGNOSTICS state = {0};
    assert(!state.muted);
    b4_diagnostics_stop(&state);
    assert(state.muted);
    b4_diagnostics_rearm(&state, 0U); /* Failed or merely requested START. */
    assert(state.muted);
    assert(!b4_diagnostics_error(B4_LSM303DLHC_STATUS_OK, 0, B4_EVENT_STATUS_OK, 0));
    assert(!b4_diagnostics_error(B4_LSM303DLHC_STATUS_NO_FRESH_DATA, 0, B4_EVENT_STATUS_OK, 0));
    assert(b4_diagnostics_error(B4_LSM303DLHC_STATUS_IO_ERROR, 0, B4_EVENT_STATUS_OK, 0));
    assert(b4_diagnostics_error(B4_LSM303DLHC_STATUS_OK, -1, B4_EVENT_STATUS_OK, 0));
    assert(b4_diagnostics_error(B4_LSM303DLHC_STATUS_OK, 0, B4_EVENT_STATUS_INVALID_EVENT, 0));
    assert(b4_diagnostics_error(B4_LSM303DLHC_STATUS_OK, 0, B4_EVENT_STATUS_OK, 1));
    b4_diagnostics_rearm(&state, 1U);
    assert(!state.muted);
    b4_diagnostics_stop(&state); /* A second hazard mutes again. */
    assert(state.muted);
    puts("Buddy 4 diagnostic gating tests passed");
    return 0;
}
