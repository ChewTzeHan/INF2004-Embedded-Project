#include "encoder.h"
#include "hardware/gpio.h"
#include "pico/time.h"

void encoder_init(bool pull_up) {
    gpio_init(ENCODER_GPIO);
    gpio_set_dir(ENCODER_GPIO, GPIO_IN);
    if (pull_up) gpio_pull_up(ENCODER_GPIO);
    else gpio_disable_pulls(ENCODER_GPIO);
}

int32_t encoder_measure_high_pulse_us(uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - t0) > timeout_us) return -1;
    }
    while (!gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - t0) > timeout_us) return -1;
    }
    uint32_t start = time_us_32();
    while (gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - start) > timeout_us) return -1;
    }
    uint32_t end = time_us_32();
    return (int32_t)(end - start);
}

int32_t encoder_measure_period_us(uint32_t timeout_us) {
    uint32_t t0 = time_us_32();
    while (gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - t0) > timeout_us) return -1;
    }
    while (!gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - t0) > timeout_us) return -1;
    }
    uint32_t first = time_us_32();
    while (gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - first) > timeout_us) return -1;
    }
    while (!gpio_get(ENCODER_GPIO)) {
        if ((time_us_32() - first) > timeout_us) return -1;
    }
    uint32_t second = time_us_32();
    return (int32_t)(second - first);
}
