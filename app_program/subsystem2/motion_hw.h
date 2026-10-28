#ifndef S2_MOTION_HW_H
#define S2_MOTION_HW_H
#include "subsystem2.h"
#define S2_GPIO_EDGE_RISE 8u
void s2_hw_safe_outputs(void);
void s2_hw_pwm_init(void);
void s2_hw_pwm_write(unsigned pin,unsigned level);
uint64_t s2_hw_time_us(void);
ER s2_hw_encoder_init(void);
/* TODO TEAM: call this ON CORE 1 from a common IO_BANK0 dispatcher when sharing IRQ21.
 * Handles/acknowledges only GP0/GP2 A rising edges; leaves other sources untouched. */
void s2_encoder_dispatch(void);
#endif
