
#include "ir_sensor.h"
#include "hardware/adc.h"
#include <stdbool.h>
#include <sys/time.h>

void ir_init(ir_calib_t *cal) {
    adc_init();
    adc_gpio_init(IR_GPIO);
    adc_select_input(IR_ADC_INPUT);
    if (cal) {
        cal->min_raw = 4095;
        cal->max_raw = 0;
    }
}

uint16_t ir_read_raw(void) {
    (void)adc_read();
    sleep_us(5);
    const int N = 8;
    uint32_t acc = 0;
    for (int i = 0; i < N; i++) {
        acc += adc_read();
        sleep_us(5);
    }
    return (uint16_t)(acc / N);
}

bool ir_is_black(uint16_t sample, const ir_calib_t *cal) {
    uint16_t th = ir_threshold(cal);
#if IR_BLACK_IS_LOWER
    return sample < th;
#else
    return sample > th;
#endif
}

int ir_classify(uint16_t sample, const ir_calib_t *cal) {
    return ir_is_black(sample, cal) ? 1 : 0;
}

int ir_classify_hysteresis(uint16_t sample, const ir_calib_t *cal, int prev_state) {
    uint16_t th = ir_threshold(cal);
    uint16_t low = (th > IR_HYST_MARGIN) ? (th - IR_HYST_MARGIN) : 0;
    uint16_t high = (th + IR_HYST_MARGIN <= 400) ? (th + IR_HYST_MARGIN) : 400;

#if IR_BLACK_IS_LOWER
    // BLACK when below 'low', WHITE when above 'high', otherwise keep previous state
    if (sample <= low) return 1;          // definitely black
    if (sample >= high) return 0;         // definitely white
    return prev_state;
#else
    // Inverted behaviour
    if (sample >= high) return 1;         // definitely black (higher = black)
    if (sample <= low) return 0;          // definitely white
    return prev_state;
#endif
}

int classify_colour(uint16_t raw){
    if (raw > 1000) return 1; // Black
    else if (raw > 400 && raw < 1000) return 2; // Grey
    else return 0; // White
}
