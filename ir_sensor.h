
#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

#ifdef __cplusplus
extern "C" {
#endif
// Add these definitions to ir_sensor.h
#ifndef RIGHT_IR_DIGITAL_PIN
#define RIGHT_IR_DIGITAL_PIN 22  // GPIO pin for right IR digital output
#endif
// === Pin selection ===
// GPIO26->ADC0, GPIO27->ADC1, GPIO28->ADC2
#ifndef IR_ADC_INPUT
#define IR_ADC_INPUT 0
#endif

#ifndef IR_GPIO
#define IR_GPIO 26
#endif

// === Behaviour switches ===
// Many reflect sensors output LOWER voltage on BLACK (less reflect).
// If your sensor is opposite (WHITE is lower), set this to 0 to invert.
#ifndef IR_BLACK_IS_LOWER
#define IR_BLACK_IS_LOWER 1
#endif

// Hysteresis margin in ADC counts (to avoid flicker around threshold).
#ifndef IR_HYST_MARGIN
#define IR_HYST_MARGIN 50// ~50/4096 ≈ 1.2%
#endif

typedef struct {
    uint16_t min_raw;
    uint16_t max_raw;
} ir_calib_t;

// Init ADC + seed calibration
void ir_init(ir_calib_t *cal);

// Read one 12-bit sample with small averaging
uint16_t ir_read_raw(void);

// Update observed min/max
static inline void ir_update_calibration(ir_calib_t *cal, uint16_t v) {
    if (v < cal->min_raw) cal->min_raw = v;
    if (v > cal->max_raw) cal->max_raw = v;
}

// Midpoint threshold
static inline uint16_t ir_threshold(const ir_calib_t *cal) {
    return (uint16_t)((cal->min_raw + cal->max_raw) / 2);
}

// Simple classification using single threshold
bool ir_is_black(uint16_t sample, const ir_calib_t *cal);
int  ir_classify(uint16_t sample, const ir_calib_t *cal); // 1=black, 0=white

// Hysteresis classification to reduce flicker.
// Provide previous state (0=white,1=black) to apply low/high bands around threshold.
int  ir_classify_hysteresis(uint16_t sample, const ir_calib_t *cal, int prev_state);

#ifdef __cplusplus
}
#endif

#endif // IR_SENSOR_H
