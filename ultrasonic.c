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
    const int MAX_ATTEMPTS = 3;
    int attempt;
    
    for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        // Send a 10 us trigger pulse to start a measurement
        gpio_put(TRIG_PIN, 0);
        sleep_us(2);
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);
        
        // Check the initial logic level of the ECHO pin before the pulse
        int raw_state_before = gpio_get(ECHO_PIN);

        // Wait until the ECHO pin goes HIGH, meaning a pulse has started
        uint32_t start_wait = time_us_32();
        while (!gpio_get(ECHO_PIN)) {
            if (time_us_32() - start_wait > 30000) {
                // printf("[DEBUG] Attempt %d: No echo HIGH detected (timeout)\n", attempt + 1);
                continue; // Try again on next attempt
            }
        }

        // Record how long the ECHO pin remains HIGH
        uint32_t start_pulse = time_us_32();
        while (gpio_get(ECHO_PIN)) {
            if (time_us_32() - start_pulse > 60000) {
                // printf("[DEBUG] Attempt %d: Echo HIGH too long (timeout)\n", attempt + 1);
                break; // Break out of this attempt
            }
        }
        uint32_t end_pulse = time_us_32();

        // Calculate pulse width and corresponding distance
        uint32_t echo_us = end_pulse - start_pulse;
        float distance_cm = (echo_us * SPEED_OF_SOUND_CM_PER_US) / 2.0f;

        // Validate the reading
        if (distance_cm >= 2.0f && distance_cm <= 400.0f) {
            // printf("[DEBUG] Attempt %d: Success - Distance: %.2f cm\n", attempt + 1, distance_cm);
            return distance_cm; // Valid reading found
        } else {
            // printf("[DEBUG] Attempt %d: Invalid distance %.2f cm\n", attempt + 1, distance_cm);
        }
        
        // Small delay between attempts to let the sensor settle
        if (attempt < MAX_ATTEMPTS - 1) {
            sleep_us(10); // 1ms delay between attempts
        }
    }
    
    // All attempts failed
    // printf("[DEBUG] All %d attempts failed to get valid distance\n", MAX_ATTEMPTS);
    return -1;
}

// Add this function to ultrasonic.c
void ultrasonic_trigger_measurement(void) {
    // Send trigger pulse
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
}


