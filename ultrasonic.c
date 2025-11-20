/**
 * ultrasonic.c - HC-SR04 ultrasonic sensor driver for distance measurement.
 */

#include "ultrasonic.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdbool.h>
#include <stdio.h>

// Conversion constant: speed of sound in air (343.2 m/s) expressed in cm per microsecond
#define SPEED_OF_SOUND_CM_PER_US 0.03432f

void ultrasonic_init(void)
{
    // Configure the TRIG pin as output
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0); // ensure low at startup

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
float ultrasonic_get_distance_cm(void)
{
    const int MAX_ATTEMPTS = 3;
    int       attempt;

    for (attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        // Send a 10 us trigger pulse to start a measurement
        // printf("[DEBUG] Attempt %d: Triggering measurement\n", attempt + 1);

        // Ensure TRIG is low for at least 2us before triggering
        gpio_put(TRIG_PIN, 0);
        sleep_us(5);

        // Send trigger pulse
        gpio_put(TRIG_PIN, 1);
        sleep_us(12); // Slightly longer pulse for reliability
        gpio_put(TRIG_PIN, 0);

        // Wait for ECHO to go HIGH with timeout
        uint32_t start_wait = time_us_32();
        uint32_t timeout_us = 30000; // 30ms timeout

        while (!gpio_get(ECHO_PIN)) {
            if (time_us_32() - start_wait > timeout_us) {
                // printf("[DEBUG] Attempt %d: Timeout waiting for ECHO HIGH\n", attempt + 1);
                break; // Break out of ECHO wait loop
            }
        }

        // If we timed out waiting for ECHO HIGH, try again
        if (time_us_32() - start_wait > timeout_us) {
            sleep_ms(50); // Wait before retry
            continue;
        }

        // ECHO is HIGH - measure pulse duration
        uint32_t pulse_start   = time_us_32();
        uint32_t pulse_timeout = 60000; // 60ms timeout (max ~10m distance)

        while (gpio_get(ECHO_PIN)) {
            if (time_us_32() - pulse_start > pulse_timeout) {
                // printf("[DEBUG] Attempt %d: ECHO pulse too long\n", attempt + 1);
                break;
            }
        }

        uint32_t pulse_end      = time_us_32();
        uint32_t pulse_duration = pulse_end - pulse_start;

        // Calculate distance
        float distance_cm = (pulse_duration * SPEED_OF_SOUND_CM_PER_US) / 2.0f;

        // Validate reading
        if (distance_cm >= 2.0f && distance_cm <= 400.0f) {
            // printf("[DEBUG] Attempt %d: Success - Distance: %.2f cm\n", attempt + 1,
            // distance_cm);
            return distance_cm;
        } else {
            // printf("[DEBUG] Attempt %d: Invalid distance %.2f cm\n", attempt + 1, distance_cm);
        }

        // Wait before next attempt
        sleep_ms(60); // HC-SR04 needs ~60ms between measurements
    }

    // printf("[DEBUG] All %d attempts failed\n", MAX_ATTEMPTS);
    return -1;
}
// Add this function to ultrasonic.c
void ultrasonic_trigger_measurement(void)
{
    // Send trigger pulse
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
}

// Add to ultrasonic.c
// Fast obstacle detection - single attempt, quick timeout
bool ultrasonic_detect_obstacle_fast(void)
{
    // Send trigger pulse
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // Wait for ECHO to go HIGH with short timeout
    uint32_t start_wait = time_us_32();
    uint32_t timeout_us = 5000; // 5ms timeout (shorter for fast detection)

    while (!gpio_get(ECHO_PIN)) {
        if (time_us_32() - start_wait > timeout_us) {
            return false; // No echo received
        }
    }

    // ECHO is HIGH - measure pulse duration
    uint32_t pulse_start   = time_us_32();
    uint32_t pulse_timeout = 10000; // 10ms timeout (max ~1.7m distance)

    while (gpio_get(ECHO_PIN)) {
        if (time_us_32() - pulse_start > pulse_timeout) {
            return false; // Echo too long
        }
    }

    uint32_t pulse_end      = time_us_32();
    uint32_t pulse_duration = pulse_end - pulse_start;

    // Calculate distance
    float distance_cm = (pulse_duration * SPEED_OF_SOUND_CM_PER_US) / 2.0f;

    // Return true if obstacle is within detection range
    return (distance_cm >= 2.0f && distance_cm <= 20.0f); // Using 20cm as detection threshold
}
