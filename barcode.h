/**
 * barcode.h - Header file for Code 39 barcode scanning and decoding.
 */

#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Max decoded payload length (not counting start/stop and checksum)
#ifndef BARCODE_MAX_LENGTH
#define BARCODE_MAX_LENGTH 32
#endif

// Max number of captured bar/space durations in one scan
#ifndef MAX_TRANSITIONS
#define MAX_TRANSITIONS 256
#endif

// Digital IR sensor pin
#ifndef RIGHT_IR_DIGITAL_PIN
#define RIGHT_IR_DIGITAL_PIN 22
#endif

// Interrupt capture control
#define BARCODE_QUIET_US 50000u // 50 ms quiet period to end barcode

typedef enum {
    BARCODE_CMD_UNKNOWN = 0,
    BARCODE_CMD_LEFT,
    BARCODE_CMD_RIGHT,
    BARCODE_CMD_STOP,
    BARCODE_CMD_FORWARD,
} barcode_command_t;

typedef struct {
    bool     valid;
    bool     checksum_ok; // Code 39 Mod 43 validation result
    char     data[BARCODE_MAX_LENGTH + 1];
    uint8_t  length;
    uint32_t scan_time_us;
} barcode_result_t;

// Interrupt capture state
typedef struct {
    volatile bool     capturing;
    volatile uint32_t last_edge_us;
    volatile uint16_t count;
    volatile uint32_t durations[MAX_TRANSITIONS];
    volatile bool     frame_ready;
} barcode_capture_state_t;

// Initialize barcode subsystem with digital IR sensor
void barcode_init(void);

// Enable IRQ capture on the digital IR pin (rising+falling edges)
void barcode_irq_init(void);

// Get the current capture state
barcode_capture_state_t* barcode_get_capture_state(void);

// Non-blocking capture flow:
bool barcode_capture_ready(void);
bool barcode_decode_captured(barcode_result_t* result);

// Digital blocking scan (uses digital pin with polling) - keep for compatibility
bool barcode_scan_digital(barcode_result_t* result);

// Interrupt-based scanning functions
bool barcode_scan_interrupt(barcode_result_t* result);
void barcode_reset_capture(void);

// Other existing functions...
bool barcode_scan_while_moving(barcode_result_t* result, char* nw_pattern, size_t pattern_size,
                               char* timing_str, size_t timing_size, char* direction_str,
                               size_t direction_size);
bool decode_with_sliding_window(const uint32_t* dur, uint16_t n, barcode_result_t* result,
                                char* all_decoded_values, size_t decoded_size);
barcode_command_t barcode_parse_command(const char* barcode_str);
// Add these to barcode.h
bool check_barcode_detection(void);
bool barcode_scan_only(barcode_result_t* result, char* nw_pattern, size_t pattern_size,
                       char* timing_str, size_t timing_size, char* direction_str,
                       size_t direction_size);
#ifdef __cplusplus
}
#endif

#endif // BARCODE_H
