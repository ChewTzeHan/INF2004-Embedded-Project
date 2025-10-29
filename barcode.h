#ifndef BARCODE_H
#define BARCODE_H

#include <stdint.h>
#include <stdbool.h>

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

typedef enum {
    BARCODE_CMD_UNKNOWN = 0,
    BARCODE_CMD_LEFT,
    BARCODE_CMD_RIGHT,
    BARCODE_CMD_STOP,
    BARCODE_CMD_FORWARD,
} barcode_command_t;

typedef struct {
    bool     valid;
    bool     checksum_ok;            // Code 39 Mod 43 validation result
    char     data[BARCODE_MAX_LENGTH + 1];
    uint8_t  length;
    uint32_t scan_time_us;
} barcode_result_t;

// Initialize barcode subsystem with digital IR sensor
void barcode_init(void);

// Enable IRQ capture on the digital IR pin (rising+falling edges)
void barcode_irq_init(void);

// Non-blocking capture flow:
// - Call barcode_capture_ready() in a timer/task to see if a frame is ready.
// - If ready, call barcode_decode_captured(&result) to decode and consume it.
bool barcode_capture_ready(void);
bool barcode_decode_captured(barcode_result_t *result);

// Digital blocking scan (uses digital pin with polling)
bool barcode_scan_digital(barcode_result_t *result);

// Map decoded string to a motion command
barcode_command_t barcode_parse_command(const char *barcode_str);

#ifdef __cplusplus
}
#endif

#endif // BARCODE_H