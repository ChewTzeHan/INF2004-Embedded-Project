#ifndef MOTOR_ENCODER_DEMO_H
#define MOTOR_ENCODER_DEMO_H

#include <stdint.h>

// Encoder accumulator structure definition
typedef struct {
    volatile uint32_t last_rise;
    volatile uint32_t last_fall;
    volatile uint32_t period_sum;
    volatile uint32_t high_sum;
    volatile uint32_t edges;
    volatile uint32_t highs;
    uint32_t last_state;  // ADD THIS - for polling
} enc_acc_t;

// External encoder structures
extern enc_acc_t E1, E2;

// Function declarations
void enc1_poll(void);  // CHANGE: polling instead of ISR
void enc2_poll(void);  // CHANGE: polling instead of ISR
void setup_pwm(uint pin);
void set_pwm_pct(uint pin, float pct);
void drive_signed(float left_pct, float right_pct);
void all_stop(void);
void enc_reset_move(void);
void run_for(float left_pct, float right_pct, uint32_t duration_ms);
void print_motor_help(void);
void motor_encoder_init(void);
void process_motor_command(char* line);
void motor_encoder_poll(void);  // ADD THIS - main polling function

#endif // MOTOR_ENCODER_DEMO_H