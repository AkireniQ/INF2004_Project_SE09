#ifndef S5_DRV_ULTRASONIC_H
#define S5_DRV_ULTRASONIC_H

#include "s5_range.h"

bool s5_hw_init(void);
bool s5_hw_init_ranging(void);
void s5_hw_echo_irq(void);
unsigned s5_servo_pulse_us(unsigned angle);
void s5_hw_angle(unsigned angle);
S5_PING s5_hw_ping(void);
S5_PING s5_hw_ping_extended(void);

#endif
