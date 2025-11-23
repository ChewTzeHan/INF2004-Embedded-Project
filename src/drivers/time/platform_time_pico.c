/*
  platform_time_pico.c
  Small helpers for platform_millis and platform_delay_ms on Raspberry Pi Pico SDK.
*/

#include <stdint.h>
#include "pico/time.h"
#include "pico/stdlib.h"

uint64_t platform_millis_pico(void) {
    return time_us_64() / 1000ULL;
}

void platform_delay_ms_pico(uint32_t ms) {
    sleep_ms(ms);
}