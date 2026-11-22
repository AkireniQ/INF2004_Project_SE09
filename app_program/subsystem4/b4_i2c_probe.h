/*
 * b4_i2c_probe.h
 * Buddy 4 IMU runtime and test-build I2C diagnostics.
 */

#ifndef B4_I2C_PROBE_H
#define B4_I2C_PROBE_H

#include <stdint.h>

#include "../app.h"

/* Start the single-owner 100 Hz acquisition and processing runtime. */
IMPORT ER b4_runtime_start(void);
IMPORT uint8_t b4_runtime_is_started(void);

#if APP_TEST
/* Test-only diagnostic logger task. */
IMPORT void b4_i2c_log_task(INT stacd, void *exinf);
#endif

#endif /* B4_I2C_PROBE_H */
