/*
 * b4_i2c_probe.h
 * Test-build-only I2C identification probe for Buddy 4.
 */

#ifndef B4_I2C_PROBE_H
#define B4_I2C_PROBE_H

#include "../app.h"

#if APP_TEST
/*
 * Open I2C0 and report candidate accelerometer identity and magnetometer
 * address responses. A responding address alone is not model confirmation.
 */
IMPORT ER b4_i2c_probe_run(void);

/* Start the periodic hardware probe task in test builds only. */
IMPORT ER b4_i2c_probe_start(void);

/* Test-only task entry point. */
IMPORT void b4_i2c_probe_task(INT stacd, void *exinf);
#endif

#endif /* B4_I2C_PROBE_H */
