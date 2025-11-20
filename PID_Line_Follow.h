/**
 * PID_Line_Follow.h - Header file for PID-based line following control.
 */

#ifndef PID_LINE_FOLLOW_H
#define PID_LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

// ==============================
// CONSTANTS
// ==============================

#define CONSTANT_SPEED 30.0f // Constant motor speed percentage
// ==============================
// FUNCTION DECLARATIONS
// ==============================

// Simple line following functions
void follow_line_simple(void);
void set_motor_constant_speed(void);

// Test function for standalone testing
void line_follow_test(void);
void follow_line_simple_with_params(float base_left_speed, float base_right_speed,
                                    float max_correction);

#endif // PID_LINE_FOLLOW_H
