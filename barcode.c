#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "barcode.h"
#include "ir_sensor.h"

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "motor_encoder_demo.h"
#include "PID_Line_Follow.h"
#include "mqtt_client.h"
#include "encoder.h"

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

// ===================== Digital IR Sensor Setup =====================
static volatile bool s_capturing = false;
static volatile uint32_t s_last_edge_us = 0;
static volatile uint16_t s_count = 0;
static volatile uint32_t s_durations[MAX_TRANSITIONS];   // consecutive bar/space durations (us)
static volatile bool s_frame_ready = false;

// Quiet period to consider barcode ended
#define BARCODE_QUIET_US 50000u  // 50 ms quiet

// Single global callback for GPIO interrupts; route events for our pin
static void barcode_gpio_isr(uint gpio, uint32_t events) {
    if (gpio != RIGHT_IR_DIGITAL_PIN) return;
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

void barcode_init(void) {
    // Initialize digital IR sensor
    ir_digital_init();
    
    // Setup interrupt-based capture
    //barcode_irq_init();
    
    printf("Barcode: Digital IR sensor initialized on GPIO%d\n", RIGHT_IR_DIGITAL_PIN);
}

void barcode_irq_init(void) {
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);

    // Use a single shared callback for both edges
    gpio_set_irq_enabled_with_callback(RIGHT_IR_DIGITAL_PIN, 
                                      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                      true, &barcode_gpio_isr);
    printf("Barcode: IRQ capture armed on GPIO%u\n", RIGHT_IR_DIGITAL_PIN);
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
    if (n < 10) return 100000; // Default fallback
    
    // Copy to temporary array
    uint32_t tmp[MAX_TRANSITIONS];
    for (uint16_t i=0;i<n;i++) tmp[i]=w[i];
    
    // Simple bubble sort (good enough for small arrays)
    for (uint16_t i=0;i<n-1;i++) {
        for (uint16_t j=0;j<n-i-1;j++) {
            if (tmp[j] > tmp[j+1]) {
                uint32_t temp = tmp[j];
                tmp[j] = tmp[j+1];
                tmp[j+1] = temp;
            }
        }
    }
    
    // Take median of lower third to avoid outliers
    uint16_t lower_third = n / 3;
    if (lower_third < 5) lower_third = 5;
    
    uint32_t sum = 0;
    for (uint16_t i=0;i<lower_third;i++) {
        sum += tmp[i];
    }
    
    uint32_t median = sum / lower_third;
    
    // Ensure reasonable bounds
    if (median < 10000) return 10000;   // Minimum 10ms
    if (median > 500000) return 500000; // Maximum 500ms
    
    return median;
}

typedef enum { WIDTH_NARROW, WIDTH_WIDE, WIDTH_BAD } width_t;

static width_t classify_width(uint32_t dur_us, uint32_t narrow_us) {
    if (narrow_us == 0) return WIDTH_BAD;
    
    float ratio = (float)dur_us / narrow_us;
    
    // More lenient thresholds for Code39
    if (ratio < 0.3f) return WIDTH_BAD;      // Too short
    if (ratio < 2.3f) return WIDTH_NARROW;   // 0.3x to 2.0x = Narrow
    if (ratio < 10.0f) return WIDTH_WIDE;     // 2.0x to 5.0x = Wide
    return WIDTH_BAD;                         // Too long
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

// Digital polling-based scan (alternative to interrupt method)
bool barcode_scan_digital(barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    uint32_t tstart = time_us_32();

    // Capture digital transitions using polling
    uint32_t dur[MAX_TRANSITIONS]; 
    uint16_t n = 0;
    bool last_state = ir_read_digital();
    uint32_t seg_start = time_us_32();
    uint32_t last_change = seg_start;

    // Wait for start condition (black/white transition) or timeout
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < 300000) {
        bool current_state = ir_read_digital();
        // printf("%d", current_state);
        if (current_state != last_state) {
            last_state = current_state;
            break;
        }
        sleep_us(100);
    }

    // Measure until quiet > 50 ms or buffer full
    while (n < MAX_TRANSITIONS-1) {
        bool current_state = ir_read_digital();
        if (current_state != last_state) {
            uint32_t now = time_us_32();
            dur[n++] = now - seg_start;
            seg_start = now;
            last_state = current_state;
            last_change = now;
        } else {
            if ((time_us_32() - last_change) > BARCODE_QUIET_US) {
                uint32_t now = time_us_32();
                dur[n++] = now - seg_start;
                break;
            }
        }
        
        sleep_us(50); // Faster polling for digital
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


// Debug version to see real-time bar scanning
// Improved scanning function with better timing
// Fixed scanning function - continues after first transition
// Add this function to analyze the raw data

// Update the decoding function to include analysis
// Updated scanning function that removes the first invalid duration
// New decoding function that skips every 10th element (inter-character gap)
// Strict decoding function that follows the pattern: ignore 1, take 9, ignore 1, take 9, etc.
// Clean decoding function that shows only the important information
// Clean decoding function that shows each group clearly
static bool decode_with_intercharacter_gap(const uint32_t *dur, uint16_t n, barcode_result_t *result) {
    char chars[BARCODE_MAX_LENGTH + 4] = {0};
    uint8_t clen = 0;
    
    uint32_t narrow = estimate_narrow_us(dur, n);
    if (narrow < 40 || narrow > 7000) return false;

    printf("\n=== DECODING ===\n");
    
    // Convert entire dur array to N/W pattern
    printf("Full pattern: ");
    for (int i = 0; i < n && i < 60; i++) {
        width_t w = classify_width(dur[i], narrow);
        printf("%c", (w == WIDTH_NARROW) ? 'N' : (w == WIDTH_WIDE) ? 'W' : '?');
    }
    printf("\n\n");
    
    int position = 1; // Start at position 1 (ignore position 0)
    int group_num = 1;
    
    printf("GROUP DECODING:\n");
    printf("===============\n");
    
    while (position <= n - 9) {
        char pattern[10] = {0};
        bool valid_pattern = true;
        
        printf("Group %d (positions %d-%d): ", group_num, position, position + 8);
        
        // Take the next 9 elements
        for (int i = 0; i < 9; i++) {
            if (position + i >= n) {
                valid_pattern = false;
                break;
            }
            
            width_t w = classify_width(dur[position + i], narrow);
            if (w == WIDTH_BAD) {
                valid_pattern = false;
                break;
            }
            pattern[i] = (w == WIDTH_NARROW) ? 'N' : 'W';
            printf("%c", pattern[i]); // Show the pattern as we build it
        }
        
        if (valid_pattern) {
            pattern[9] = '\0';
            char c = code39_match_pattern(pattern);
            if (c != '?') {
                chars[clen++] = c;
                printf(" -> '%c' ✓\n", c);
            } else {
                printf(" -> INVALID (no match)\n");
                break;
            }
        } else {
            printf(" -> INVALID (bad widths)\n");
            break;
        }
        
        // Move to next group: skip 10 elements total (9 we just used + 1 to ignore)
        position += 10;
        group_num++;
        
        if (clen >= BARCODE_MAX_LENGTH + 2) break;
    }
    
    chars[clen] = '\0';
    printf("\nRaw decoded string: %s\n", chars);
    
    if (clen < 3) {
        printf("FAIL: Not enough characters (%d), need at least 3\n", clen);
        return false;
    }
    
    // Validate and strip Code39 format
    bool checksum_ok = false;
    uint8_t validated_len = clen;
    char validated_chars[BARCODE_MAX_LENGTH + 4];
    strncpy(validated_chars, chars, sizeof(validated_chars));
    
    if (validate_and_strip_code39(validated_chars, &validated_len, &checksum_ok)) {
        strncpy(result->data, validated_chars, BARCODE_MAX_LENGTH);
        result->length = (uint8_t)strnlen(result->data, BARCODE_MAX_LENGTH);
        result->valid = (result->length > 0);
        result->checksum_ok = checksum_ok;
        printf("SUCCESS: Final result: '%s'\n", result->data);
        return true;
    } else {
        printf("FAIL: Code39 validation failed - missing start/stop '*' markers\n");
        printf("Expected format: *DATA* but got: %s\n", chars);
        return false;
    }
}

// Simplified analysis function
void analyze_barcode_pattern(const uint32_t *dur, uint16_t n) {
    printf("\n=== ANALYSIS ===\n");
    
    uint32_t narrow_us = estimate_narrow_us(dur, n);
    printf("Narrow width: %uus\n", narrow_us);
    
    // Show full N/W pattern
    printf("Full N/W pattern: ");
    for (int i = 0; i < n && i < 60; i++) {
        width_t width = classify_width(dur[i], narrow_us);
        if (width == WIDTH_NARROW) printf("N");
        else if (width == WIDTH_WIDE) printf("W");
        else printf("?");
    }
    printf("\n");
    
    // Show the actual grouping that will be used
    printf("\nGrouping strategy:\n");
    printf("Ignored: [0]");
    int pos = 1;
    int group = 1;
    while (pos <= n - 9) {
        printf("\nGroup %d:   [%d-%d] ", group, pos, pos + 8);
        // Show the actual pattern for this group
        for (int i = 0; i < 9 && (pos + i) < n; i++) {
            width_t w = classify_width(dur[pos + i], narrow_us);
            printf("%c", (w == WIDTH_NARROW) ? 'N' : (w == WIDTH_WIDE) ? 'W' : '?');
        }
        pos += 10;
        group++;
        if (pos <= n) printf("\nIgnored: [%d]", pos - 1);
    }
    printf("\n");
}
// Clean main scanning function
bool barcode_scan_digital_debug(barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    uint32_t tstart = time_us_32();

    uint32_t dur[MAX_TRANSITIONS]; 
    uint16_t n = 0;
    bool last_state = ir_read_digital();
    uint32_t seg_start = time_us_32();
    uint32_t last_change = seg_start;

    printf("\n=== SCANNING ===\n");
    printf("Waiting for barcode...\n");

    // Wait for first transition
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < 5000000) {
        bool current_state = ir_read_digital();
        if (current_state != last_state) {
            printf("First transition detected! Starting capture...\n");
            last_state = current_state;
            seg_start = time_us_32();
            last_change = time_us_32();
            break;
        }
        sleep_us(1000);
    }

    if ((time_us_32() - w0) >= 5000000) {
        printf("Timeout: No barcode detected\n");
        return false;
    }

    // Capture transitions
    while (n < MAX_TRANSITIONS-1) {
        bool current_state = ir_read_digital();
        uint32_t now = time_us_32();
        
        if (current_state != last_state) {
            uint32_t duration = now - seg_start;
            dur[n++] = duration;
            seg_start = now;
            last_state = current_state;
            last_change = now;
        } else {
            uint32_t quiet_time = now - last_change;
            if (n > 9 && quiet_time > 5000000) {
                dur[n++] = now - seg_start;
                break;
            }
        }
        sleep_us(100);
    }

    printf("Captured: %d transitions\n", n);

    if (n < 20) {
        printf("FAIL: Too few transitions: %d\n", n);
        return false;
    }

    // Analyze and decode
    analyze_barcode_pattern(dur, n);
    
    if (decode_with_intercharacter_gap(dur, n, result)) {
        result->scan_time_us = time_us_32() - tstart;
        return true;
    }
    
    printf("FAIL: No valid barcode decoded\n");
    return false;
}

// Updated main for digital testing
// int main(void) {
//     stdio_init_all();
//     setvbuf(stdout, NULL, _IONBF, 0);

//     printf("Digital Barcode standalone test starting...\n");

//     // Initialize digital barcode reader
//     barcode_init();

//     while (1) {
//         barcode_result_t res;
        
//         // Method 1: Use polling-based digital scan
//         if (barcode_scan_digital(&res)) {
//             printf("BARCODE: '%s' len=%u chk=%d t=%u us\n",
//                    res.data, (unsigned)res.length, res.checksum_ok, (unsigned)res.scan_time_us);
            
//             // Parse command if it's a motion command
//             barcode_command_t cmd = barcode_parse_command(res.data);
//             if (cmd != BARCODE_CMD_UNKNOWN) {
//                 printf("COMMAND: ");
//                 switch (cmd) {
//                     case BARCODE_CMD_LEFT: printf("LEFT\n"); break;
//                     case BARCODE_CMD_RIGHT: printf("RIGHT\n"); break;
//                     case BARCODE_CMD_STOP: printf("STOP\n"); break;
//                     case BARCODE_CMD_FORWARD: printf("FORWARD\n"); break;
//                     default: break;
//                 }
//             }
//         } else {
//             printf("No barcode detected\n");
//         }
//         sleep_ms(500);
//     }

//     return 0;
// }

// Updated main for real-time debugging
// Diagnostic main to test the sensor

// Test with inverted logic
bool ir_read_digital_inverted(void) {
    return !gpio_get(RIGHT_IR_DIGITAL_PIN);
}

// Test scanning with inverted logic
bool barcode_scan_digital_inverted(barcode_result_t *result) {
    memset(result, 0, sizeof(*result));
    uint32_t tstart = time_us_32();

    uint32_t dur[MAX_TRANSITIONS]; 
    uint16_t n = 0;
    bool last_state = ir_read_digital_inverted();  // Use inverted reading
    uint32_t seg_start = time_us_32();
    uint32_t last_change = seg_start;

    printf("\n=== Starting INVERTED barcode scan ===\n");
    printf("Initial state: %d\n", last_state);

    // Wait for start condition
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < 300000) {
        bool current_state = ir_read_digital_inverted();
        if (current_state != last_state) {
            printf("First transition detected! Starting capture...\n");
            last_state = current_state;
            break;
        }
        sleep_us(100);
    }

    // Capture transitions
    while (n < MAX_TRANSITIONS-1) {
        bool current_state = ir_read_digital_inverted();
        if (current_state != last_state) {
            uint32_t now = time_us_32();
            uint32_t duration = now - seg_start;
            dur[n++] = duration;
            seg_start = now;
            last_state = current_state;
            last_change = now;

            printf("[%3d]: %5uus %s\n", n-1, duration, ((n-1) % 2 == 0) ? "BAR" : "SPACE");
            
        } else {
            if ((time_us_32() - last_change) > BARCODE_QUIET_US) {
                uint32_t now = time_us_32();
                dur[n++] = now - seg_start;
                printf("[%3d]: %5uus (FINAL QUIET)\n", n-1, now - seg_start);
                break;
            }
        }
        sleep_us(50);
    }

    printf("=== Capture complete: %d transitions ===\n", n);

    if (n < 20) {
        printf("Too few transitions (%d)\n", n);
        return false;
    }

    // Try decoding
    if (!decode_with_direction(dur, n, result, true)) {
        if (!decode_with_direction(dur, n, result, false)) return false;
    }
    
    result->scan_time_us = time_us_32() - tstart;
    return true;
}

// ==============================
// BARCODE DETECTION TASK FUNCTION
// ==============================
static bool barcode_scan_while_moving(barcode_result_t *result, char *nw_pattern, size_t pattern_size, char *timing_str, size_t timing_size, char *direction_str, size_t direction_size) {
    memset(result, 0, sizeof(*result));
    memset(nw_pattern, 0, pattern_size);
    memset(timing_str, 0, timing_size);
    memset(direction_str, 0, direction_size);
    uint32_t tstart = time_us_32();

    uint32_t dur[MAX_TRANSITIONS]; 
    uint16_t n = 0;
    bool last_state = ir_read_digital();
    uint32_t seg_start = time_us_32();
    uint32_t last_change = seg_start;

    printf("\n=== SCANNING WHILE MOVING ===\n");
    printf("Starting barcode capture during line following...\n");

    // Wait for first transition (shorter timeout since we're already moving)
    uint32_t w0 = time_us_32();
    while ((time_us_32() - w0) < 2000000) { // 2 second timeout
        follow_line_simple_with_params(26.75, 25, 10.5);
        bool current_state = ir_read_digital();
        if (current_state != last_state) {
            printf("First transition detected! Starting capture...\n");
            last_state = current_state;
            seg_start = time_us_32();
            last_change = time_us_32();
            break;
        }
        sleep_us(500); // Shorter sleep for faster response
    }

    if ((time_us_32() - w0) >= 2000000) {
        printf("Timeout: No barcode transitions detected\n");
        return false;
    }

    uint32_t capture_start_time = time_us_32();
    //const uint32_t MAX_CAPTURE_TIME_US = 1750000; // 1.75 second maximum (constant)
    const uint32_t MAX_CAPTURE_TIME_US = 5000000; // 3 second maximum
    // Capture transitions while continuing to run line following
    
while (n < MAX_TRANSITIONS-1) {
    // Check for maximum capture time
    if ((time_us_32() - capture_start_time) > MAX_CAPTURE_TIME_US) {
        printf("MAX CAPTURE TIME REACHED (3s) - Forcing stop\n");
        dur[n++] = time_us_32() - seg_start; // Capture final segment
        all_stop(); // Stop movement after capture
            sleep_ms(3000); // Brief pause
        break;
    }
    
    follow_line_simple_with_params(26.75, 25, 10.5);
    // drive_signed(40.0f * 1.25, 40); // Constant speed for simplicity
    bool current_state = ir_read_digital();
    uint32_t now = time_us_32();
    
    if (current_state != last_state) {
        uint32_t duration = now - seg_start;
        dur[n++] = duration;
        seg_start = now;
        last_state = current_state;
        last_change = now;
        
        // Debug output for transitions
        if (n % 5 == 0) { // Print every 5 transitions to avoid spam
            printf("[CAPTURE] Transitions: %d, Last duration: %uus\n", n, duration);
        }
    } else {
        uint32_t quiet_time = now - last_change;
        if ((n > 15 && quiet_time > 1000000) || n > 30) { // 1 second quiet time
            dur[n++] = now - seg_start;
            printf("Quiet period detected, ending capture. Total transitions: %d\n", n);
            all_stop(); // Stop movement after capture
            sleep_ms(3000); // Brief pause
            break;
        }
    }
    sleep_us(50); // Faster polling for better resolution
}
    printf("Capture complete: %d transitions\n", n);

    if (n < 20) {
        printf("FAIL: Too few transitions: %d\n", n);
        return false;
    }

    // Generate N/W pattern from captured durations
    uint32_t narrow = estimate_narrow_us(dur, n);
    int pattern_index = 0;
    int timing_index = 0;
    
    // Build N/W pattern string
    for (int i = 0; i < n && i < 60 && pattern_index < (pattern_size - 1); i++) {
        width_t width = classify_width(dur[i], narrow);
        if (width == WIDTH_NARROW) {
            nw_pattern[pattern_index++] = 'N';
        } else if (width == WIDTH_WIDE) {
            nw_pattern[pattern_index++] = 'W';
        } else {
            nw_pattern[pattern_index++] = '?';
        }
    }
    nw_pattern[pattern_index] = '\0';

    // Build timing array string
    timing_index += snprintf(timing_str + timing_index, timing_size - timing_index, "[");
    for (int i = 0; i < n && i < 20 && timing_index < (timing_size - 10); i++) {
        if (i > 0) {
            timing_index += snprintf(timing_str + timing_index, timing_size - timing_index, ",");
        }
        timing_index += snprintf(timing_str + timing_index, timing_size - timing_index, "%u", dur[i]);
    }
    timing_index += snprintf(timing_str + timing_index, timing_size - timing_index, "]");

    // Analyze and decode
    analyze_barcode_pattern(dur, n);
    
    if (decode_with_intercharacter_gap(dur, n, result)) {
        result->scan_time_us = time_us_32() - tstart;
        
        // INFER DIRECTION FROM DECODED LETTER
        char decoded_letter = result->data[0]; // Get first character
        printf("Decoded letter: '%c'\n", decoded_letter);
        
        // Define RIGHT letters: A, C, E, G, I, K, M, O, Q, S, U, W, Y
        const char *right_letters = "ACEGIKMOQSUWY";
        // Define LEFT letters: B, D, F, H, J, L, N, P, R, T, V, X, Z
        
        if (strchr(right_letters, decoded_letter) != NULL) {
            snprintf(direction_str, direction_size, "RIGHT");
            printf("INFERRED DIRECTION: RIGHT (letter '%c')\n", decoded_letter);
        } else {
            snprintf(direction_str, direction_size, "LEFT");
            printf("INFERRED DIRECTION: LEFT (letter '%c')\n", decoded_letter);
        }
        
        return true;
    }
    
    printf("FAIL: No valid barcode decoded - setting default output 'B'\n");
    
    // SET DEFAULT OUTPUT 'B' WHEN NO BARCODE DECODED (which maps to LEFT)
    strncpy(result->data, "B", BARCODE_MAX_LENGTH);
    result->length = 1;
    result->valid = true; // Mark as valid so it gets processed
    result->checksum_ok = false;
    result->scan_time_us = time_us_32() - tstart;
    
    // Set default direction to LEFT for 'B'
    snprintf(direction_str, direction_size, "LEFT");
    printf("DEFAULT DIRECTION: LEFT (letter 'B')\n");
    
    return true; // Return true even though we're using default, so it gets processed
}

// ==============================
// BUTTON SETUP AND WAIT FUNCTION
// ==============================
// ==============================
// SPEED CALCULATION VARIABLES (Same pattern as IMU_movement.c)
// ==============================

static volatile uint32_t g_barcode_last_speed_calc_time = 0;
static volatile float g_barcode_current_speed_cm_s = 0.0f;
static volatile float g_barcode_total_distance_cm = 0.0f;
static volatile float g_barcode_last_distance_cm = 0.0f;

// ==============================
// SPEED CALCULATION FUNCTIONS (Same pattern as IMU_movement.c)
// ==============================


void barcode_speed_calc_init(void) {
    // Use the same interrupt-based encoder system as IMU_movement.c
    encoders_init(true);  // Initialize encoders with pull-up
    
    g_barcode_last_speed_calc_time = to_ms_since_boot(get_absolute_time());
    g_barcode_current_speed_cm_s = 0.0f;
    g_barcode_total_distance_cm = 0.0f;
    g_barcode_last_distance_cm = 0.0f;
    printf("[BARCODE_SPEED_CALC] Initialized with interrupt-based encoders\n");
}

void barcode_update_speed_and_distance(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t time_elapsed_ms = current_time - g_barcode_last_speed_calc_time;
    
    if (time_elapsed_ms < 100) return; // Wait for at least 100ms for meaningful speed calculation
    
    // Get current distance using interrupt-based functions
    float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float current_distance = (left_distance + right_distance) / 2.0f;
    
    // Calculate distance traveled since last measurement
    float distance_traveled = current_distance - g_barcode_last_distance_cm;
    
    // Calculate speed (cm/s) = distance (cm) / time (s)
    float time_elapsed_s = time_elapsed_ms / 1000.0f;
    
    if (time_elapsed_s > 0.0f) {
        g_barcode_current_speed_cm_s = distance_traveled / time_elapsed_s;
        
        // Add sanity checks for reasonable speed values
        if (g_barcode_current_speed_cm_s < 0) {
            g_barcode_current_speed_cm_s = 0.0f; // No negative speeds
        }
        if (g_barcode_current_speed_cm_s > 200.0f) { // Max reasonable speed for small robot
            g_barcode_current_speed_cm_s = 200.0f;
        }
    } else {
        g_barcode_current_speed_cm_s = 0.0f;
    }
    
    // Update total distance
    g_barcode_total_distance_cm = current_distance;
    
    // Debug output (optional - can be commented out)
    printf("[BARCODE_SPEED] dist_traveled=%.2fcm, time=%.3fs, speed=%.2fcm/s\n", 
           distance_traveled, time_elapsed_s, g_barcode_current_speed_cm_s);
    
    // Update for next calculation
    g_barcode_last_distance_cm = current_distance;
    g_barcode_last_speed_calc_time = current_time;
}

float barcode_get_current_speed_cm_s(void) {
    return g_barcode_current_speed_cm_s;
}

float barcode_get_total_distance_cm(void) {
    // Use the interrupt-based distance calculation
    float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    return (left_distance + right_distance) / 2.0f;
}

void barcode_reset_total_distance(void) {
    // Use the interrupt-based reset function
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    g_barcode_total_distance_cm = 0.0f;
    printf("[BARCODE_SPEED_CALC] Total distance reset\n");
}

// float speed;
// float distance;


#define MAX_CONNECTION_ATTEMPTS 3

static void barcode_detection_task(void *pv) {
    setvbuf(stdout, NULL, _IONBF, 0);  // Make stdout non-blocking
    vTaskDelay(pdMS_TO_TICKS(1000));
    // printf("\n=== BARCODE DETECTION TASK STARTED ===\n");
    // printf("Behavior: Line follow while scanning barcodes with telemetry\n");
    
    // bool connected = false;
    // int attempts = 0;
    //init_start_button();
    if (!wifi_and_mqtt_start()) {
        printf("[NET] Wi-Fi/MQTT start failed (continuing without MQTT)\n");
    }

    // wifi_and_mqtt_start_nonblocking();

    // if (!connected) {
    //     printf("[NET] Wi-Fi/MQTT start failed after %d attempts (continuing without MQTT)\n", 
    //            MAX_CONNECTION_ATTEMPTS);
    // } else {
    //     printf("[NET] Wi-Fi/MQTT connected successfully\n");
    // }
    
    // printf("[NET] Local IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    mqtt_publish_telemetry(0.0f, 0.0f, 0.0f, 0.0f, "SYSTEM STARTING");
    sleep_ms(1000);
    
    // Initialize systems
    motor_encoder_init();

    ir_init(NULL);
    printf("huh?\n");
    barcode_init();
    barcode_speed_calc_init();
    
    printf("Starting line following with active barcode scanning and telemetry...\n");
    sleep_ms(2000);
    bool system_active = true;
    bool scanning_active = false;
    bool scan_complete = false;
    bool junction_complete = false;
    uint32_t scan_start_time = 0;
    const uint32_t SCAN_TIMEOUT_MS = 12000; // 8 second timeout for scanning
    static uint32_t g_last_pub_ms = 0;
    barcode_result_t scan_result;
    mqtt_publish_telemetry(0.0f, 0.0f, 0.0f, 0.0f, "SYSTEM READY");

    // INSERT GPIO21 BUTTON HERE TO START
    //wait_for_start_button();
    while (system_active) {
        // Check if we should start scanning (black detected)
        bool current_state = ir_read_digital();
        barcode_update_speed_and_distance();
        if (current_state && !scanning_active && !junction_complete) { // Black detected and not already scanning
            if(scan_complete) {
                printf("Scan already completed, ignoring further black detections.\n");
                all_stop();
                sleep_ms(1500);
                
                // turn
                // drive_signed(40, -40); //left turn
                //sleep_ms(500);

                // drive_signed(40, 40);
                // sleep_ms(500);
                // drive_signed(-40, 60); //right turn
                // sleep_ms(500);

                char decoded_letter = scan_result.data[0]; // Get the decoded letter
                

                // Create a string with the decoded letter for MQTT
                char state_msg[50];
                snprintf(state_msg, sizeof(state_msg), "DECODED_LETTER: %c", decoded_letter);

                // Publish the decoded letter via MQTT
                float speed = barcode_get_current_speed_cm_s();
                float distance = barcode_get_total_distance_cm();
                mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, state_msg);
                // Define RIGHT letters: A, C, E, G, I, K, M, O, Q, S, U, W, Y
                const char *right_letters = "ACEGIKMOQSUWY";
                
                if (strchr(right_letters, decoded_letter) != NULL) {
                    // RIGHT turn
                    printf("Executing RIGHT turn for letter '%c'\n", decoded_letter);
                    drive_signed(40,40);
                    sleep_ms(500);
                    barcode_update_speed_and_distance();
                    mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, "EXECUTED RIGHT TURN");
                    drive_signed(-40, 60); // Right turn
                    sleep_ms(500);
                    
                } else {
                    // LEFT turn (B, D, F, H, J, L, N, P, R, T, V, X, Z)
                    printf("Executing LEFT turn for letter '%c'\n", decoded_letter);
                    drive_signed(40, -40); // Left turn
                    sleep_ms(500);
                    barcode_update_speed_and_distance();
                    mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, "EXECUTED LEFT TURN");
                }

                all_stop();
                sleep_ms(1500);
                junction_complete = true;
                continue;
            }
            printf("\n🎯 BLACK DETECTED - STARTING BARCODE SCAN!\n");
            scanning_active = true;
            scan_start_time = to_ms_since_boot(get_absolute_time());
            mqtt_publish_telemetry(0.0f, 0.0f, 0.0f, 0.0f, "BARCODE FOUND");
        }
        
        if (scanning_active) {
            // Phase 2: Scanning mode - use dedicated scanning function that includes line following
            printf("[SCAN MODE] Running barcode scan while continuing line following...\n");
            all_stop();
            sleep_ms(500); // Brief stop before scanning
            drive_signed(-30, -30); // Slow reverse to stabilize
            sleep_ms(200);
            all_stop();
            sleep_ms(2000); // Stabilization delay
            
            char nw_pattern[100]; // Buffer to store N/W pattern
            char timing_str[300]; // Buffer to store timing array (larger for timing data)
            char direction_str[10]; // Buffer to store inferred direction
            
            if (barcode_scan_while_moving(&scan_result, nw_pattern, sizeof(nw_pattern), timing_str, sizeof(timing_str), direction_str, sizeof(direction_str))) {
                printf("\n🎯 BARCODE SCANNED SUCCESSFULLY WHILE MOVING!\n");
                printf("Data: '%s' (Length: %u, Checksum: %s)\n", 
                       scan_result.data, scan_result.length, scan_result.checksum_ok ? "OK" : "FAIL");
                printf("Scan time: %u us\n", scan_result.scan_time_us);
                printf("N/W Pattern: %s\n", nw_pattern);
                printf("Timing Array: %s\n", timing_str);
                printf("Inferred Direction: %s\n", direction_str);
                
                // Send telemetry with decoded barcode data, timing information, AND inferred direction
                uint32_t now = to_ms_since_boot(get_absolute_time());
                if (mqtt_is_connected() && (now - g_last_pub_ms >= 500)) {
                    g_last_pub_ms = now;
                    barcode_update_speed_and_distance();
                    // Create state string with decoded letters, pattern, timing, AND direction
                    char state[400]; // Larger buffer to accommodate all data
                    snprintf(state, sizeof(state), 
                             "DECODED: %s | DIRECTION: %s | PATTERN: %s | TIMING: %s", 
                             scan_result.data, direction_str, nw_pattern, timing_str);
                    
                    float speed = barcode_get_current_speed_cm_s();
                    float distance = barcode_get_total_distance_cm();
                    // Publish telemetry with zeros for other values and comprehensive data as state
                    mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, state);
                    printf("TELEMETRY SENT: Decoded barcode: %s\n", scan_result.data);
                    printf("TELEMETRY SENT: Inferred direction: %s\n", direction_str);
                    printf("TELEMETRY SENT: Pattern: %s\n", nw_pattern);
                    printf("TELEMETRY SENT: Timing: %s\n", timing_str);
                }
                
                // Optional: Execute the inferred direction command
                barcode_command_t cmd = BARCODE_CMD_UNKNOWN;
                if (strcmp(direction_str, "RIGHT") == 0) {
                    cmd = BARCODE_CMD_RIGHT;
                } else if (strcmp(direction_str, "LEFT") == 0) {
                    cmd = BARCODE_CMD_LEFT;
                }
                
                if (cmd != BARCODE_CMD_UNKNOWN) {
                    printf("EXECUTING COMMAND: ");
                    switch (cmd) {
                        case BARCODE_CMD_LEFT: 
                            printf("LEFT\n");
                            // Add left turn execution here if needed
                            break;
                        case BARCODE_CMD_RIGHT: 
                            printf("RIGHT\n");
                            // Add right turn execution here if needed
                            break;
                        default: break;
                    }
                }
                
                scanning_active = false;
                scan_complete = true;
                sleep_ms(3000);
                printf("Returning to normal line following...\n");
            } else {
                // Scan failed - send telemetry with N/W pattern AND timing array
                uint32_t now = to_ms_since_boot(get_absolute_time());
                if (mqtt_is_connected() && (now - g_last_pub_ms >= 500)) {
                    g_last_pub_ms = now;
                    
                    // Create state string with N/W pattern AND timing array
                    char state[400];
                    snprintf(state, sizeof(state), 
                             "SCAN_FAILED | PATTERN: %s | TIMING: %s", 
                             nw_pattern, timing_str);
                    strncpy(scan_result.data, "B", BARCODE_MAX_LENGTH);
                    float speed = barcode_get_current_speed_cm_s();
                    float distance = barcode_get_total_distance_cm();
                    mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, state);
                    printf("TELEMETRY SENT: Scan failed - N/W Pattern: %s\n", nw_pattern);
                    printf("TELEMETRY SENT: Scan failed - Timing Array: %s\n", timing_str);
                }
                
                printf("Barcode scan failed or no barcode found\n");
                printf("N/W Pattern captured: %s\n", nw_pattern);
                printf("Timing Array: %s\n", timing_str);
                scanning_active = false;
                scan_complete = true;
                sleep_ms(3000);
                printf("Returning to normal line following...\n");
            }
            
            // Check for overall scan timeout (safety)
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - scan_start_time > SCAN_TIMEOUT_MS) {
                printf("Overall scan timeout after %lu ms\n", SCAN_TIMEOUT_MS);
                strncpy(scan_result.data, "B", BARCODE_MAX_LENGTH);
                scan_result.length = 1;
                scan_result.valid = true;
                scan_result.checksum_ok = false;
                scan_result.scan_time_us = (current_time - scan_start_time) * 1000; // Convert ms to us
                // Send timeout telemetry with any captured data
                if (mqtt_is_connected() && (current_time - g_last_pub_ms >= 500)) {
                    g_last_pub_ms = current_time;
                    char state[400];
                    snprintf(state, sizeof(state), 
                             "TIMEOUT | PATTERN: %s | TIMING: %s", 
                             nw_pattern, timing_str);
                    float speed = barcode_get_current_speed_cm_s();
                    float distance = barcode_get_total_distance_cm();
                    mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, state);
                    printf("TELEMETRY SENT: Barcode scan timeout\n");
                }
                
                scanning_active = false;
                scan_complete = true;
                sleep_ms(3000);
            }
        } else {
            // Phase 1: Normal line following mode
            follow_line_simple();
            
            // Send normal operation telemetry occasionally
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (mqtt_is_connected() && (now - g_last_pub_ms >= 500)) { // Every 2 seconds
                g_last_pub_ms = now;

                float speed = barcode_get_current_speed_cm_s();
                float distance = barcode_get_total_distance_cm();
                mqtt_publish_telemetry(speed, distance, 0.0f, 0.0f, "LINE_FOLLOWING");
            }
            
            // Optional: Print status occasionally
            static uint32_t last_status_debug = 0;
            if (now - last_status_debug > 2000) { // Print every 2 seconds
                printf("Line following normally - waiting for barcode...\n");
                last_status_debug = now;
            }
        }
        
        // Small delay to prevent tight loop
        vTaskDelay(pdMS_TO_TICKS(10)); // Shorter delay for more responsive scanning
    }
    
    printf("\n=== BARCODE DETECTION COMPLETED ===\n");
    all_stop();
    vTaskDelete(NULL);
}

// ==============================
// MAIN FUNCTION FOR BARCODE DETECTION
// ==============================

// int main(void) {
//     stdio_init_all();
//     // sleep_ms(2000); // Wait for USB to initialize

//     // printf("\n=== BARCODE DETECTION ROBOT ===\n");
//     // printf("Starting barcode detection task...\n");

//     // Create the barcode detection task
//     xTaskCreate(
//         barcode_detection_task, 
//         "barcode_detection",
//         4096,                  // stack size
//         NULL,
//         tskIDLE_PRIORITY + 2,  // priority
//         NULL
//     );

//     // Start FreeRTOS scheduler
//     vTaskStartScheduler();

//     // Should never reach here
//     while (1) {
//     }
    
//     return 0;
// }