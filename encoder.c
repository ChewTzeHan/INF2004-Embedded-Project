/**
 * encoder.c - Wheel encoder driver with interrupt-based pulse counting and distance tracking.
 */

#include "encoder.h"
#include "hardware/gpio.h"
#include "motor_encoder_demo.h"
#include "pico/time.h"

// Global encoder instances
encoder_t left_encoder  = {ENCODER_LEFT_GPIO, 0, 0, 0, 0.0f};
encoder_t right_encoder = {ENCODER_RIGHT_GPIO, 0, 0, 0, 0.0f};

// Global interrupt handler that routes to the correct encoder
void encoder_global_isr(uint gpio, uint32_t events)
{
    uint32_t current_time = time_us_32();

    if (gpio == ENCODER_LEFT_GPIO && (events & GPIO_IRQ_EDGE_RISE)) {
        // Left encoder rising edge
        left_encoder.pulse_count++;

        if (left_encoder.last_time_us > 0) {
            uint32_t period = current_time - left_encoder.last_time_us;

            // Add bounds checking - reasonable pulse periods for a wheel
            if (period > 1000 && period < 1000000) { // 1ms to 1 second range
                left_encoder.last_period_us = period;

                // Calculate distance per pulse (wheel circumference / pulses per revolution)
                float distance_per_pulse = (WHEEL_DIAMETER_CM * 3.14159f) / PULSES_PER_REVOLUTION;
                left_encoder.distance_cm += distance_per_pulse;
            }
        }
        left_encoder.last_time_us = current_time;
    } else if (gpio == ENCODER_RIGHT_GPIO && (events & GPIO_IRQ_EDGE_RISE)) {
        // Right encoder rising edge
        right_encoder.pulse_count++;

        if (right_encoder.last_time_us > 0) {
            uint32_t period = current_time - right_encoder.last_time_us;

            // Add bounds checking - reasonable pulse periods for a wheel
            if (period > 1000 && period < 1000000) { // 1ms to 1 second range
                right_encoder.last_period_us = period;

                // Calculate distance per pulse (wheel circumference / pulses per revolution)
                float distance_per_pulse = (WHEEL_DIAMETER_CM * 3.14159f) / PULSES_PER_REVOLUTION;
                right_encoder.distance_cm += distance_per_pulse;
            }
        }
        right_encoder.last_time_us = current_time;
    }
}

void encoder_init(bool pull_up)
{
    gpio_init(ENCODER_LEFT_GPIO);
    gpio_set_dir(ENCODER_LEFT_GPIO, GPIO_IN);
    gpio_init(ENCODER_RIGHT_GPIO);
    gpio_set_dir(ENCODER_RIGHT_GPIO, GPIO_IN);

    if (pull_up) {
        gpio_pull_up(ENCODER_LEFT_GPIO);
        gpio_pull_up(ENCODER_RIGHT_GPIO);
    } else {
        gpio_disable_pulls(ENCODER_LEFT_GPIO);
        gpio_disable_pulls(ENCODER_RIGHT_GPIO);
    }
}

void encoders_init(bool pull_up)
{
    encoder_init(pull_up);

    // Initialize encoder structures with sane defaults
    uint32_t initial_time       = time_us_32();
    left_encoder.gpio           = ENCODER_LEFT_GPIO;
    left_encoder.pulse_count    = 0;
    left_encoder.last_time_us   = initial_time;
    left_encoder.last_period_us = 100000; // Start with a reasonable default (0.1s period = 10cm/s)
    left_encoder.distance_cm    = 0.0f;

    right_encoder.gpio           = ENCODER_RIGHT_GPIO;
    right_encoder.pulse_count    = 0;
    right_encoder.last_time_us   = initial_time;
    right_encoder.last_period_us = 100000; // Start with a reasonable default
    right_encoder.distance_cm    = 0.0f;

    // Clear any pending interrupts
    gpio_acknowledge_irq(ENCODER_LEFT_GPIO, GPIO_IRQ_EDGE_RISE);
    gpio_acknowledge_irq(ENCODER_RIGHT_GPIO, GPIO_IRQ_EDGE_RISE);

    // Set up interrupts using the global handler
    gpio_set_irq_enabled(ENCODER_LEFT_GPIO, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(ENCODER_RIGHT_GPIO, GPIO_IRQ_EDGE_RISE, true);

    // Set the global callback
    gpio_set_irq_callback(encoder_global_isr);

    // Enable GPIO interrupts
    irq_set_enabled(IO_IRQ_BANK0, true);
}

int32_t encoder_measure_high_pulse_us(uint gpio_pin, uint32_t timeout_us)
{
    uint32_t t0 = time_us_32();
    while (gpio_get(gpio_pin)) {
        if ((time_us_32() - t0) > timeout_us)
            return -1;
    }
    while (!gpio_get(gpio_pin)) {
        if ((time_us_32() - t0) > timeout_us)
            return -1;
    }
    uint32_t start = time_us_32();
    while (gpio_get(gpio_pin)) {
        if ((time_us_32() - start) > timeout_us)
            return -1;
    }
    uint32_t end = time_us_32();
    return (int32_t)(end - start);
}

int32_t encoder_measure_period_us(uint gpio_pin, uint32_t timeout_us)
{
    uint32_t t0 = time_us_32();
    while (gpio_get(gpio_pin)) {
        if ((time_us_32() - t0) > timeout_us)
            return -1;
    }
    while (!gpio_get(gpio_pin)) {
        if ((time_us_32() - t0) > timeout_us)
            return -1;
    }
    uint32_t first = time_us_32();
    while (gpio_get(gpio_pin)) {
        if ((time_us_32() - first) > timeout_us)
            return -1;
    }
    while (!gpio_get(gpio_pin)) {
        if ((time_us_32() - first) > timeout_us)
            return -1;
    }
    uint32_t second = time_us_32();
    return (int32_t)(second - first);
}

// Empty function - interrupts handle everything
void encoder_update_measurements(void)
{
    // Interrupts handle all measurements automatically
}

float encoder_get_distance_cm(uint gpio_pin)
{
    encoder_t* encoder = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    return encoder->distance_cm;
}

int32_t encoder_get_pulse_count(uint gpio_pin)
{
    encoder_t* encoder = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    return encoder->pulse_count;
}

void encoder_reset_distance(uint gpio_pin)
{
    encoder_t* encoder      = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;
    encoder->distance_cm    = 0.0f;
    encoder->pulse_count    = 0;
    encoder->last_time_us   = time_us_32();
    encoder->last_period_us = 0;
}

float encoder_get_speed_cm_s(uint gpio_pin)
{
    encoder_t* encoder = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;

    // Add proper bounds checking
    if (encoder->last_period_us <= 1000 || encoder->last_period_us > 10000000) {
        return 0.0f; // Return 0 for unreasonable period values
    }

    float distance_per_pulse = (WHEEL_DIAMETER_CM * 3.14159f) / PULSES_PER_REVOLUTION;
    float time_per_pulse_s   = encoder->last_period_us / 1e6f;

    // Add sanity check to prevent division by extremely small numbers
    if (time_per_pulse_s < 0.0001f) { // Less than 0.1ms period
        return 0.0f;
    }

    float speed_cm_s = distance_per_pulse / time_per_pulse_s;

    // Add maximum speed limit (unlikely to exceed 200 cm/s for a small robot)
    if (speed_cm_s > 200.0f) {
        return 200.0f;
    }

    return speed_cm_s;
}

float encoder_get_speed_cm_s_timeout(uint gpio_pin, uint32_t timeout_ms)
{
    encoder_t* encoder = (gpio_pin == ENCODER_LEFT_GPIO) ? &left_encoder : &right_encoder;

    uint32_t current_time          = time_us_32();
    uint32_t time_since_last_pulse = (current_time - encoder->last_time_us) / 1000;

    // If no pulses received for timeout period, return 0 speed
    if (time_since_last_pulse > timeout_ms) {
        return 0.0f;
    }

    // Use the main speed calculation function which now has bounds checking
    return encoder_get_speed_cm_s(gpio_pin);
}

// Standalone test main
#include <stdio.h>

// int main() {
//     stdio_init_all();
//     printf("Encoder Test Starting (Interrupts Only)...\n");
//     sleep_ms(4000);

//     // Initialize encoders with interrupts
//     encoders_init(true);

//     // Initialize motors
//     motor_encoder_init();

//     printf("Testing both encoders with interrupts only...\n");
//     printf("Left encoder GPIO: %d, Right encoder GPIO: %d\n", ENCODER_LEFT_GPIO,
//     ENCODER_RIGHT_GPIO);

//     // Test both motors briefly to see if encoders work
//     drive_signed(30.0f, 30.0f);
//     sleep_ms(1000);

//     // Check initial counts
//     printf("Initial counts - Left: %ld, Right: %ld\n",
//            left_encoder.pulse_count, right_encoder.pulse_count);

//     drive_signed(40.0f, 40.0f);

//     uint32_t last_print_time = time_us_32();

//     while (true) {
//         uint32_t current_time = time_us_32();

//         // Print status every 500ms
//         if (current_time - last_print_time > 500000) {
//             float left_speed = encoder_get_speed_cm_s_timeout(ENCODER_LEFT_GPIO, 200);
//             float right_speed = encoder_get_speed_cm_s_timeout(ENCODER_RIGHT_GPIO, 200);
//             float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
//             float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
//             int32_t left_pulses = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
//             int32_t right_pulses = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);

//             printf("Left: Speed=%.2f cm/s, Distance=%.2f cm, Pulses=%ld\n",
//                    left_speed, left_distance, left_pulses);
//             printf("Right: Speed=%.2f cm/s, Distance=%.2f cm, Pulses=%ld\n",
//                    right_speed, right_distance, right_pulses);
//             printf("---\n");

//             last_print_time = current_time;
//         }

//         sleep_ms(1000);
//     }

//     return 0;
// }
