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
    barcode_irq_init();
    
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
    if (ratio < 2.0f) return WIDTH_NARROW;   // 0.3x to 2.0x = Narrow
    if (ratio < 5.0f) return WIDTH_WIDE;     // 2.0x to 5.0x = Wide
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
            if (n > 15 && quiet_time > 5000000) {
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

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("=== IMPROVED BARCODE SCANNER ===\n");
    printf("Now with longer timeouts and better feedback\n");

    // Initialize digital barcode reader
    barcode_init();

    while (1) {
        barcode_result_t res;
        
        printf("\n" "======" "\n");
        printf("Ready to scan - move barcode slowly in front of sensor\n");
        printf("You have 5 seconds to start scanning...\n");
        
        if (barcode_scan_digital_debug(&res)) {
            printf("\n*** BARCODE SUCCESS ***\n");
            printf("Data: '%s' len=%u checksum=%d\n",
                   res.data, (unsigned)res.length, res.checksum_ok);
            
            // Parse command if it's a motion command
            barcode_command_t cmd = barcode_parse_command(res.data);
            if (cmd != BARCODE_CMD_UNKNOWN) {
                printf("COMMAND: ");
                switch (cmd) {
                    case BARCODE_CMD_LEFT: printf("LEFT\n"); break;
                    case BARCODE_CMD_RIGHT: printf("RIGHT\n"); break;
                    case BARCODE_CMD_STOP: printf("STOP\n"); break;
                    case BARCODE_CMD_FORWARD: printf("FORWARD\n"); break;
                    default: break;
                }
            }
        } else {
            printf("\n*** Scan failed or no barcode detected ***\n");
        }
        
        printf("Waiting 3 seconds before next scan...\n");
        sleep_ms(3000);
    }

    return 0;
}