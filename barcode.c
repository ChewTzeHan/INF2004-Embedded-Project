#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "barcode.h"
#include "ir_sensor.h"

// ===================== Code 39 table (with Mod 43 values) =====================
typedef struct {
    char character;
    const char *pattern;  // 9 symbols: W/N alternating bar/space starting with bar
    int  value;           // Mod 43 value; '*' has no value
} code39_char_t;

static const code39_char_t code39_table[] = {
    {'0',"NNNWWNWNN", 0},  {'1',"WNNWNNNNW", 1}, {'2',"NNWWNNNNW", 2},
    {'3',"WNWWNNNNN", 3},  {'4',"NNNWWNNNW", 4}, {'5',"WNNWWNNNN", 5},
    {'6',"NNWWWNNNN", 6},  {'7',"NNNWNNWNW", 7}, {'8',"WNNWNNWNN", 8},
    {'9',"NNWWNNWNN", 9},  {'A',"WNNNNWNNW",10}, {'B',"NNWNNWNNW",11},
    {'C',"WNWNNWNNN",12},  {'D',"NNNNWWNNW",13}, {'E',"WNNNWWNNN",14},
    {'F',"NNWNWWNNN",15},  {'G',"NNNNNWWNW",16}, {'H',"WNNNNWWNN",17},
    {'I',"NNWNNWWNN",18},  {'J',"NNNNWWWNN",19}, {'K',"WNNNNNNWW",20},
    {'L',"NNWNNNNWW",21},  {'M',"WNWNNNNWN",22}, {'N',"NNNNWNNWW",23},
    {'O',"WNNNWNNWN",24},  {'P',"NNWNWNNWN",25}, {'Q',"NNNNNNWWW",26},
    {'R',"WNNNNNWWN",27},  {'S',"NNWNNNWWN",28}, {'T',"NNNNWNWWN",29},
    {'U',"WWNNNNNWN",30},  {'V',"NWWNNNNWN",31}, {'W',"WWWNNNNNN",32},
    {'X',"NWNNWNNWN",33},  {'Y',"WWNNWNNNN",34}, {'Z',"NWWNWNNNN",35},
    {'-',"NWNNNNWWN",36},  {'.',"WWNNNNNWW",37}, {' ',"NWWNNNWNN",38},
    {'$',"NWNWNNWNN",39},  {'/',"NWNNNWNWN",40}, {'+',"NNWNWNWNN",41},
    {'%',"NNNWNWNNW",42},  {'*',"NWNNWNWNN",-1}
};

#define CODE39_TABLE_SIZE (sizeof(code39_table)/sizeof(code39_char_t))

static int code39_value(char c) {
    for (uint i=0; i<CODE39_TABLE_SIZE; ++i)
        if (code39_table[i].character == c) return code39_table[i].value;
    return -1;
}
static char code39_match_pattern(const char *p) {
    for (uint i=0; i<CODE39_TABLE_SIZE; ++i)
        if (strcmp(p, code39_table[i].pattern) == 0) return code39_table[i].character;
    return '?';
}

// ===================== Local IR calibration for analog =====================
static ir_calib_t s_ir_cal;
static int s_prev_state = 0;

void barcode_init(void) {
    ir_init(&s_ir_cal);
    for (int i=0; i<40; ++i) {
        uint16_t v = ir_read_raw();
        ir_update_calibration(&s_ir_cal, v);
        sleep_ms(5);
    }
    s_prev_state = 0;
    printf("Barcode: IR calibrated, Code 39 ready\n");
}

// ===================== IRQ capture on RIGHT IR digital =====================
static uint s_bar_gpio = 0;
static volatile bool s_capturing = false;
static volatile uint32_t s_last_edge_us = 0;
static volatile uint16_t s_count = 0;
static volatile uint32_t s_durations[MAX_TRANSITIONS];   // consecutive bar/space durations (us)
static volatile bool s_frame_ready = false;

// Quiet period to consider barcode ended
#define BARCODE_QUIET_US 50000u  // 50 ms quiet

// Single global callback for GPIO interrupts; route events for our pin
static void barcode_gpio_isr(uint gpio, uint32_t events) {
    if (gpio != s_bar_gpio) return;
    uint32_t now = time_us_32();

    if (!s_capturing) {
        s_capturing = true;
        s_last_edge_us = now;
        s_count = 0;
        return;
    }

    uint32_t dt = now - s_last_edge_us;
    s_last_edge_us = now;

    if (s_count < MAX_TRANSITIONS) {
        s_durations[s_count++] = dt;
    } else {
        // Buffer full → mark ready; stop capture
        s_frame_ready = true;
        s_capturing = false;
    }
}

void barcode_irq_init(uint gpio) {
    s_bar_gpio = gpio;
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);

    // Use a single shared callback (Pico SDK supports one global or per-pin callback)
    gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &barcode_gpio_isr);
    printf("Barcode: IRQ capture armed on GPIO%u\n", gpio);
}

// Call periodically (e.g., in a 30 ms timer) to finalize capture on quiet time
bool barcode_capture_ready(void) {
    if (s_frame_ready) return true;
    if (s_capturing) {
        uint32_t quiet = time_us_32() - s_last_edge_us;
        if (quiet > BARCODE_QUIET_US && s_count >= 8) {
            s_frame_ready = true;
            s_capturing = false;
        }
    }
    return s_frame_ready;
}

// ===================== Decoding helpers =====================
static uint32_t estimate_narrow_us(const uint32_t *w, uint16_t n) {
    if (n < 10) return 0;
    // Simple partial sort of lower half for median
    uint32_t tmp[MAX_TRANSITIONS];
    for (uint16_t i=0;i<n;i++) tmp[i]=w[i];
    for (uint16_t i=1;i<n;i++){
        uint32_t key=tmp[i]; int j=(int)i-1;
        while (j>=0 && tmp[j]>key) { tmp[j+1]=tmp[j]; j--; }
        tmp[j+1]=key;
    }
    uint16_t end = n/2 ? (n/2) : n;
    uint16_t mid = end/2 ? (end/2) : 1;
    return tmp[mid];
}

typedef enum { WIDTH_NARROW, WIDTH_WIDE, WIDTH_BAD } width_t;

static width_t classify_width(uint32_t dur_us, uint32_t narrow_us) {
    if (narrow_us == 0) return WIDTH_BAD;
    if (dur_us < (uint32_t)(narrow_us * 0.5f)) return WIDTH_BAD;
    if (dur_us < (uint32_t)(narrow_us * 1.6f)) return WIDTH_NARROW;
    if (dur_us < (uint32_t)(narrow_us * 3.8f)) return WIDTH_WIDE;
    return WIDTH_BAD;
}

static bool decode_symbols_to_chars(const uint32_t *dur, uint16_t n, char *out, uint8_t *out_len, bool expect_bar_first) {
    *out_len = 0;
    if (n < 9) return false;

    uint32_t narrow = estimate_narrow_us(dur, n);
    if (narrow < 40 || narrow > 7000) return false;

    char pattern[10]; uint8_t pidx = 0;
    bool bar = expect_bar_first;

    for (uint16_t i=0;i<n;i++) {
        (void)bar; // classification independent of current color
        width_t w = classify_width(dur[i], narrow);
        if (w == WIDTH_BAD) continue;
        pattern[pidx++] = (w == WIDTH_WIDE) ? 'W' : 'N';
        if (pidx == 9) {
            pattern[9] = '\0';
            char c = code39_match_pattern(pattern);
            out[(*out_len)++] = c;
            pidx = 0;
            if (*out_len >= BARCODE_MAX_LENGTH + 2) break; // include start/stop
        }
        bar = !bar;
    }
    out[*out_len] = 0;
    return (*out_len >= 3);
}

static bool validate_and_strip_code39(char *chars, uint8_t *len, bool *checksum_ok) {
    *checksum_ok = false;
    if (*len < 3) return false;
    if (chars[0] != '*' || chars[*len - 1] != '*') return false;

    // Optional Mod 43 checksum (if >= 4 chars including *)
    if (*len >= 4) {
        int sum = 0;
        for (uint8_t i=1; i<(*len - 2); i++) {
            int v = code39_value(chars[i]);
            if (v < 0) return false;
            sum += v;
        }
        int chk = sum % 43;
        int last = code39_value(chars[*len - 2]);
        if (last >= 0 && last == chk) {
            *checksum_ok = true;
            // Strip checksum char (shift final '*' left)
            chars[*len - 2] = '*';
            chars[*len - 1] = '\0';
            (*len)--;
        }
    }

    // Strip the surrounding '*' to keep payload only
    uint8_t payload = (*len) - 2;
    memmove(chars, chars + 1, payload);
    chars[payload] = '\0';
    *len = payload;
    return true;
}

static bool decode_with_direction(const uint32_t *dur, uint16_t n, barcode_result_t *result, bool forward) {
    char chars[BARCODE_MAX_LENGTH + 4] = {0};
    uint8_t clen = 0;
    uint32_t tmp[MAX_TRANSITIONS];

    if (forward) {
        for (uint16_t i=0;i<n;i++) tmp[i]=dur[i];
    } else {
        for (uint16_t i=0;i<n;i++) tmp[i]=dur[n-1-i];
    }

    bool chk=false;
    if (decode_symbols_to_chars(tmp, n, chars, &clen, true) &&
        validate_and_strip_code39(chars, &clen, &chk)) {
        strncpy(result->data, chars, BARCODE_MAX_LENGTH);
        result->length = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid = (result->length > 0);
        result->checksum_ok = chk;
        return result->valid;
    }
    if (decode_symbols_to_chars(tmp, n, chars, &clen, false) &&
        validate_and_strip_code39(chars, &clen, &chk)) {
        strncpy(result->data, chars, BARCODE_MAX_LENGTH);
        result->length = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid = (result->length > 0);
        result->checksum_ok = chk;
        return result->valid;
    }
    return false;
}

// ===================== Public decode entry points =====================
bool barcode_decode_captured(barcode_result_t *result) {
    if (!s_frame_ready) return false;

    // Copy volatile buffer
    uint16_t n = s_count;
    if (n < 9) { s_frame_ready = false; return false; }
    uint32_t buf[MAX_TRANSITIONS];
    for (uint16_t i=0;i<n;i++) buf[i] = s_durations[i];

    // Consume frame
    s_frame_ready = false;

    uint32_t t0 = time_us_32();
    if (!decode_with_direction(buf, n, result, true)) {
        if (!decode_with_direction(buf, n, result, false)) {
            return false;
        }
    }
    result->scan_time_us = time_us_32() - t0;
    return true;
}

// Analog fallback (blocking). Uses hysteresis on analog IR.
bool barcode_scan_analog(barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    uint32_t tstart = time_us_32();

    // Capture analog transitions (simplified)
    uint32_t dur[MAX_TRANSITIONS]; uint16_t n=0;
    int last = s_prev_state;
    uint32_t seg_start = time_us_32();
    uint32_t last_change = seg_start;

    // Wait for black to start or timeout
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < 300000) {
        uint16_t raw = ir_read_raw();
        int s = ir_classify_hysteresis(raw, &s_ir_cal, last);
        if (s == 1) { last = s; break; }
        sleep_us(150);
    }

    // Measure until quiet > 50 ms or buffer full
    while (n < MAX_TRANSITIONS-1) {
        uint16_t raw = ir_read_raw();
        int s = ir_classify_hysteresis(raw, &s_ir_cal, last);
        if (s != last) {
            uint32_t now = time_us_32();
            dur[n++] = now - seg_start;
            seg_start = now;
            last = s;
            last_change = now;
        } else {
            if ((time_us_32() - last_change) > BARCODE_QUIET_US) {
                uint32_t now = time_us_32();
                dur[n++] = now - seg_start;
                break;
            }
        }
        sleep_us(80);
    }

    if (n < 20) return false;

    if (!decode_with_direction(dur, n, result, true)) {
        if (!decode_with_direction(dur, n, result, false)) return false;
    }
    result->scan_time_us = time_us_32() - tstart;
    return true;
}

barcode_command_t barcode_parse_command(const char *s) {
    if (!s || !*s) return BARCODE_CMD_UNKNOWN;
    if (!strcmp(s,"LEFT") || !strcmp(s,"L"))       return BARCODE_CMD_LEFT;
    if (!strcmp(s,"RIGHT")|| !strcmp(s,"R"))       return BARCODE_CMD_RIGHT;
    if (!strcmp(s,"STOP") || !strcmp(s,"S"))       return BARCODE_CMD_STOP;
    if (!strcmp(s,"GO")   || !strcmp(s,"F") ||
        !strcmp(s,"FWD")  || !strcmp(s,"FORWARD")) return BARCODE_CMD_FORWARD;
    return BARCODE_CMD_UNKNOWN;
}


/*
 * Optional standalone test main.
 * Define BARCODE_STANDALONE_MAIN when compiling this file into a test target
 * to build firmware that only runs the barcode scanner and prints results.
 */


// int main(void) {
//     stdio_init_all();
//     setvbuf(stdout, NULL, _IONBF, 0);

//     printf("Barcode standalone test starting...\n");

//     // Initialize IR and barcode calibration
//     barcode_init();

//     while (1) {
//         barcode_result_t res;
//         if (barcode_scan_analog(&res)) {
//             printf("BARCODE: '%s' len=%u chk=%d t=%u us\n",
//                    res.data, (unsigned)res.length, res.checksum_ok, (unsigned)res.scan_time_us);
//         } else {
//             printf("No barcode detected\n");
//         }
//         sleep_ms(500);
//     }

//     return 0;
// }

