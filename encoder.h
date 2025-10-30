#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ENCODER_GPIO
#define ENCODER_GPIO 14
#endif

void encoder_init(bool pull_up);
int32_t encoder_measure_high_pulse_us(uint32_t timeout_us);
int32_t encoder_measure_period_us(uint32_t timeout_us);

static inline float period_us_to_hz(int32_t period_us) {
    if (period_us <= 0) return 0.0f;
    return 1e6f / (float)period_us;
}

#ifdef __cplusplus
}
#endif

#endif
