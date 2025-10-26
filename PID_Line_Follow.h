#ifndef PID_LINE_FOLLOW_H
#define PID_LINE_FOLLOW_H

#include <stdint.h>
#include <stdbool.h>

// ==============================
// CONSTANTS
// ==============================

#define CONSTANT_SPEED 30.0f  // Constant motor speed percentage
// ==============================
// FUNCTION DECLARATIONS
// ==============================

// Simple line following functions
void follow_line_simple(void);
void set_motor_constant_speed(void);

// Test function for standalone testing
void line_follow_test(void);

#endif // PID_LINE_FOLLOW_H