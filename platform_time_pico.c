/**
 * platform_time_pico.c - Platform-specific time and delay functions for Raspberry Pi Pico.
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdint.h>

uint64_t platform_millis_pico(void)
{
    return time_us_64() / 1000ULL;
}

void platform_delay_ms_pico(uint32_t ms)
{
    sleep_ms(ms);
}
