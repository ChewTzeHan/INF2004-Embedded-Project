// Remove the polling functions and add interrupt-based ones
#ifndef MOTOR_ENCODER_DEMO_H
#define MOTOR_ENCODER_DEMO_H

#include <stdint.h>

// Encoder accumulator structure definition
typedef struct {
    volatile uint32_t pulse_count;
} enc_acc_t;

// External encoder structures
extern enc_acc_t E1, E2;

// Function declarations
void setup_pwm(uint pin);
void set_pwm_pct(uint pin, float pct);
void drive_signed(float left_pct, float right_pct);
void all_stop(void);
void print_motor_help(void);
void motor_encoder_init(void);
void process_motor_command(char* line);

// Remove polling functions
// void motor_encoder_poll(void);
// void enc1_poll(void);
// void enc2_poll(void);

#endif // MOTOR_ENCODER_DEMO_H