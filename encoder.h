/**
 * encoder.h - Header file for wheel encoder functionality with speed and distance tracking.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// Encoder GPIO definitions
#define ENCODER_LEFT_GPIO  4
#define ENCODER_RIGHT_GPIO 6

// Encoder structure to track state
typedef struct {
    uint              gpio;
    volatile int32_t  pulse_count;
    volatile uint32_t last_time_us;
    volatile int32_t  last_period_us;
    float             distance_cm; // Total distance traveled in cm
} encoder_t;

void    encoder_init(bool pull_up);
int32_t encoder_measure_high_pulse_us(uint gpio_pin, uint32_t timeout_us);
int32_t encoder_measure_period_us(uint gpio_pin, uint32_t timeout_us);

// New functions for speed and distance
void    encoders_init(bool pull_up);
float   encoder_get_speed_cm_s(uint gpio_pin);
float   encoder_get_distance_cm(uint gpio_pin);
void    encoder_update_measurements(void);
int32_t encoder_get_pulse_count(uint gpio_pin);
void    encoder_reset_distance(uint gpio_pin);

// Global encoder instances
extern encoder_t left_encoder;
extern encoder_t right_encoder;

static inline float period_us_to_hz(int32_t period_us)
{
    if (period_us <= 0)
        return 0.0f;
    return 1e6f / (float)period_us;
}

// Constants for wheel calculations
#define WHEEL_DIAMETER_CM     6.5f  // Adjust based on your wheel size
#define PULSES_PER_REVOLUTION 20.0f // Adjust based on your encoder

#ifdef __cplusplus
}
#endif

#endif
