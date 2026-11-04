#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

const uint TRIG_PIN = 0;
const uint ECHO_PIN = 1;

void setup_ultrasonic_pins(uint trig_pin, uint echo_pin)
{
    gpio_init(trig_pin);
    gpio_init(echo_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_set_dir(echo_pin, GPIO_IN);
}

/*
* Fire one ping and return the width of the ECHO pulse in microseconds.
*/
uint64_t get_pulse_us(uint trig_pin, uint echo_pin)
{
    absolute_time_t start_time, end_time;

    /* Ensure a clean low state, then send a proper 10us high trigger pulse. */
    gpio_put(trig_pin, 0);
    sleep_us(2);
    gpio_put(trig_pin, 1);
    sleep_us(10);
    gpio_put(trig_pin, 0);

    /* Block until ECHO goes high. See the README: this is the design flaw. */
    while (gpio_get(echo_pin) == 0)
        tight_loop_contents();
    start_time = get_absolute_time();

    /* Block until ECHO goes low again. */
    while (gpio_get(echo_pin) == 1)
        tight_loop_contents();
    end_time = get_absolute_time();

    return (uint64_t)absolute_time_diff_us(start_time, end_time);
}

/*
* Convert that pulse width to centimetres.
*
* Sound travels at roughly 343 m/s, which is 29.1 microseconds per
* centimetre. Divide by 2 because the pulse measures the round trip
* (there and back), but distance is one-way.
*/
uint64_t get_distance_cm(uint trig_pin, uint echo_pin)
{
    uint64_t pulse_us = get_pulse_us(trig_pin, echo_pin);

    return pulse_us / 29 / 2;
}

int main(void)
{
    stdio_init_all();
    sleep_ms(3000);

    setup_ultrasonic_pins(TRIG_PIN, ECHO_PIN);

    printf("Buddy 5 - testing in progress\n");

    while (true) {
        uint64_t cm = get_distance_cm(TRIG_PIN, ECHO_PIN);
        printf("distance: %llu cm\n", (unsigned long long)cm);
        sleep_ms(500);
    }
}
