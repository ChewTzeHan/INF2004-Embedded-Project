// ultrasonic.c

/*
This file contains all functions that handle ultrasonic distance
 measurement using an HC-SR04 sensor. It triggers a short pulse on
  the TRIG pin and measures how long the ECHO pin stays HIGH. That
   duration is then converted into a distance value in centimeters.
*/

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "ultrasonic.h"

// Conversion constant: speed of sound in air (343.2 m/s) expressed in cm per microsecond
#define SPEED_OF_SOUND_CM_PER_US 0.03432f

void ultrasonic_init(void) {
    // Configure the TRIG pin as output
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);  // ensure low at startup

    // Configure the ECHO pin as input
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    // The pull-down line below was used for early debugging when the echo pin was floating.
    // It has been commented out for normal operation so that real echo signals can be detected.
    // gpio_pull_down(ECHO_PIN);
}

/*
Sends a short trigger pulse to the ultrasonic sensor and measures the time
 taken for the echo to return. The function calculates and returns the
  distance in centimeters. If no echo is received within the timeout, it returns -1.
*/
float ultrasonic_get_distance_cm(void) {
    // Send a 10 us trigger pulse to start a measurement
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
    // printf("[DEBUG] Trigger pulse sent\n");
    //debug<1> Pico has just sent a short 10 µs HIGH pulse on the TRIG pin to tell the HC-SR04 sensor to start an ultrasonic ping
    // The sensor now emits an ultrasonic sound wave (at about 40 kHz) into the air.

    // Check the initial logic level of the ECHO pin before the pulse
    int raw_state_before = gpio_get(ECHO_PIN);
    // printf("[DEBUG] Raw ECHO pin before trigger: %d (0=LOW, 1=HIGH)\n", raw_state_before);
    //debug<2> 
    // after sending debug<1> (trigger pulse), there is a 150-300us delay between the end of the trigger pulse and when the sensor raises the ECHO line HIGH.
    // in this delay, the debug output should shows 0=LOW.


    // Wait until the ECHO pin goes HIGH, meaning a pulse has started
    uint32_t start_wait = time_us_32();
    while (!gpio_get(ECHO_PIN)) {
        if (time_us_32() - start_wait > 30000) {
            // printf("[DEBUG] No echo HIGH detected (timeout)\n");
            return -1;
        }
    }
    // printf("[DEBUG] Echo HIGH started\n");
    //debug<3> 
    //sent its ping and is now timing how long it takes for the echo to return.
    // echo is now high

    // Record how long the ECHO pin remains HIGH
    uint32_t start_pulse = time_us_32();
    while (gpio_get(ECHO_PIN)) {
        if (time_us_32() - start_pulse > 60000) {
            // printf("[DEBUG] Echo HIGH too long (timeout)\n");
            return -1;
        }
    }
    uint32_t end_pulse = time_us_32();
    // printf("[DEBUG] Echo ended\n");
    //debug<4> 
    //ECHO pin has returned to LOW, which signals the end of the timing pulse.
    //ico now knows how long the ECHO line stayed HIGH - that’s the total round-trip travel time of the ultrasonic wave.

    // Calculate pulse width and corresponding distance
    uint32_t echo_us = end_pulse - start_pulse;
    // printf("[DEBUG] Raw echo pulse width: %lu us\n", (unsigned long)echo_us);
    //debug<5> 
    // shows the duration (in microseconds) that the ECHO line stayed HIGH.

    float distance_cm = (echo_us * SPEED_OF_SOUND_CM_PER_US) / 2.0f;
    // printf("[DEBUG] Pulse duration: %lu us, Distance: %.2f cm\n",
    //     //debug<6> 
    //     //program has converted the measured time into a distance using
    //     //distance = (time × speed_of_sound) / 2.
    //        (unsigned long)echo_us, distance_cm);

    // Reject unrealistic readings outside the normal range of the sensor
    if (distance_cm < 2.0f || distance_cm > 400.0f)
        return -1;

    return distance_cm;
}
