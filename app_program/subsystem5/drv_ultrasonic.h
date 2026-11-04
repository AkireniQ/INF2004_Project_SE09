#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

// Change these to match your wiring
#define US_TRIG_PIN   6
#define US_ECHO_PIN   7

#define US_TIMEOUT_US 2000   // ~5 m max range; returns error past this

void  ultrasonic_init(void);
bool  ultrasonic_read_pulse_us(uint32_t *pulse_us);  // raw echo time
float ultrasonic_read_cm(void);                       // returns -1.0f on timeout

#endif