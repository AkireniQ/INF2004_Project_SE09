#ifndef MOTION_CORE_H
#define MOTION_CORE_H
#include <stdbool.h>
#include <stdint.h>
#include "subsystem2.h"
/* Level 0..4 means 0,25,50,75,100 percent of MAX_RPM (not PWM). */
void s2_core_init(void);
void s2_core_service(void); /* Owner task calls once per scheduled control release. */
bool s2_core_set_speed(unsigned level);
bool s2_core_increase_speed(void);
bool s2_core_decrease_speed(void);
bool s2_core_forward(void);
bool s2_core_backward(void);
bool s2_core_set_wheel_levels(unsigned left, unsigned right);
bool s2_core_move_forward_mm(unsigned mm);
bool s2_core_move_backward_mm(unsigned mm);
/* steps 1..8 means 45..360 degrees. No angle field is retained after conversion. */
bool s2_core_turn_left(unsigned steps);
bool s2_core_turn_right(unsigned steps);
void s2_core_stop(void);
bool s2_core_busy(void);
unsigned s2_core_faults(void);
bool s2_core_clear_faults(void); /* Only after checking the cause. Never auto-resumes. */
void s2_core_snapshot(S2_STATUS *out);
void s2_core_timeout(void);
uint32_t s2_core_timing_fault_gap_us(void); /* Latched diagnostic, does not drive motors. */
#endif
