#include "PID_Line_Follow.h"
#include <stdio.h>
#include <stdlib.h> 
#include "pico/stdlib.h"
#include "motor_encoder_demo.h"
#include "ir_sensor.h"

// ==============================
// PID CONSTANTS
// ==============================

#define KP 0.019f
#define KD 0.006f
#define KI 0.001f
#define SETPOINT 700
#define BASE_SPEED_LEFT 32.75f
#define BASE_SPEED_RIGHT 30.0f
#define MAX_CORRECTION 17.5f  // Limit how much we can adjust speeds

#define WHITE_THRESHOLD 300
#define WHITE_RAMP_RATE 0.125f  // how quickly correction scales up per cycle (0.0–1.0)

// Global variables for PID
static int16_t previous_error = 0;
static uint32_t previous_time = 0;
static float previous_correction = 0.0f;
static uint16_t previous_ir_value = SETPOINT;

// ==============================
// PID CALCULATION
// ==============================

float pid_calculation(uint16_t ir_value) {
    uint32_t current_time = time_us_32();
    float dt = (current_time - previous_time) / 1000000.0f;
    
    // Handle first call
    if (previous_time == 0) {
        dt = 0.01f;
    }
    
    // Compute error
    int16_t error = SETPOINT - ir_value;
    
    // Calculate derivative
    float derivative = (error - previous_error) / dt;
    
    // Calculate integral
    static float integral = 0;
    integral += error * dt;

    // Anti-windup clamp
    if (integral > 1000.0f) integral = 1000.0f;
    if (integral < -1000.0f) integral = -1000.0f;
    if (derivative > 1000.0f) derivative = 1000.0f;
    if (derivative < -1000.0f) derivative = -1000.0f;

    // PID calculation
    float correction = (KP * error) + (KD * derivative);
    
    

    //float correction = (KP * error) + (KD * derivative) + (KI * integral);
    // Limit correction
    if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
    if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
    
    
    
    // Update previous values
    previous_error = error;
    previous_time = current_time;
    

    if(ir_value > SETPOINT && previous_ir_value < SETPOINT){ //wrong, take previous correction
        correction = previous_correction;
    }
    previous_correction = correction;  // Store for potential reuse
    return correction;
}

// ==============================
// MOTOR CONTROL FUNCTIONS
// ==============================

void set_motor_speeds_pid(uint16_t ir_value) {
    static float white_ramp_factor = 0.0f; // Ranges from 0.0 to 1.0
    float correction = pid_calculation(ir_value);
    
    if (ir_value < WHITE_THRESHOLD) {
        // On white: gradually ramp up
        
        white_ramp_factor += WHITE_RAMP_RATE;
        if (white_ramp_factor > 1.0f) white_ramp_factor = 1.0f;

        // Scale correction by ramp factor (start small, grow over time)
        correction *= white_ramp_factor;
    } else {
        // Back on line or dark surface: reset ramp immediately
        
        white_ramp_factor = 0.0f;
    }

    // Apply ramped correction only if on white
    

    // Apply correction differentially to motors
    float left_speed = BASE_SPEED_LEFT - correction;   // Subtract correction
    float right_speed = BASE_SPEED_RIGHT + correction;  // Add correction

    // Limit motor speeds to safe range (0-100%)
    if (left_speed < 0) left_speed = 0;
    if (left_speed > 80.0f) left_speed = 80.0f;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 80.0f) right_speed = 80.0f;
    
    static uint32_t last_print = 0;
    uint32_t current_time = time_us_32();
    
    if (current_time - last_print > 100000) { // Print every 100ms
        printf("IR: %4u | Error: %4d | Correction: %6.2f\n | motor speeds: L=%.2f R=%.2f | ramp: %.2f\n", 
               ir_value, SETPOINT - ir_value, correction, left_speed, right_speed, white_ramp_factor);
        last_print = current_time;
    }
    // Set motor speeds
        drive_signed(left_speed, right_speed);
    //drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
}

// ==============================
// LINE FOLLOWING FUNCTION
// ==============================

void follow_line_simple(void) {
    // Read IR sensor
    uint16_t ir_value = ir_read_raw();
    
    // Set motor speeds based on PID
    set_motor_speeds_pid(ir_value);
    //drive_signed(10, 10);
    // Print debug info
    
    
}

#define param_kp 0.015f
#define param_ki 0.001f
#define param_kd 0.007f
#define param_setpoint 700
#define param_ramp_rate 0.25f

void follow_line_simple_with_params(float base_left_speed, float base_right_speed, float max_correction) {
    // Read IR sensor
    uint16_t ir_value = ir_read_raw();
    
    // Calculate PID correction
    uint32_t current_time = time_us_32();
    float dt = (current_time - previous_time) / 1000000.0f;
    
    // Handle first call
    if (previous_time == 0) {
        dt = 0.01f;
    }
    
    // Compute error
    int16_t error = param_setpoint - ir_value;
    
    // Calculate derivative
    float derivative = (error - previous_error) / dt;
    
    // Calculate integral
    static float integral = 0;
    integral += error * dt;

    // Anti-windup clamp
    if (integral > 1000.0f) integral = 1000.0f;
    if (integral < -1000.0f) integral = -1000.0f;
    if (derivative > 1000.0f) derivative = 1000.0f;
    if (derivative < -1000.0f) derivative = -1000.0f;

    // PID calculation
    float correction = (param_kp * error) + (param_kd * derivative);
    
    // Limit correction with custom max_correction
    if (correction > max_correction) correction = max_correction;
    if (correction < -max_correction) correction = -max_correction;
    
    // Update previous values
    previous_error = error;
    previous_time = current_time;

    if(ir_value > SETPOINT && previous_ir_value < SETPOINT) {
        correction = previous_correction;
    }
    previous_correction = correction;

    // Apply white surface ramp logic
    static float white_ramp_factor = 0.0f;
    
    if (ir_value < WHITE_THRESHOLD) {
        // On white: gradually ramp up
        white_ramp_factor += WHITE_RAMP_RATE;
        if (white_ramp_factor > 1.0f) white_ramp_factor = 1.0f;
        correction *= white_ramp_factor;
    } else {
        // Back on line or dark surface: reset ramp immediately
        white_ramp_factor = 0.0f;
    }

    // Apply correction differentially to motors with custom base speeds
    float left_speed = base_left_speed - correction;
    float right_speed = base_right_speed + correction;

    // Limit motor speeds to safe range (0-100%)
    if (left_speed < 0) left_speed = 0;
    if (left_speed > 80.0f) left_speed = 80.0f;
    if (right_speed < 0) right_speed = 0;
    if (right_speed > 80.0f) right_speed = 80.0f;
    
    static uint32_t last_print = 0;
    if (current_time - last_print > 100000) { // Print every 100ms
        printf("IR: %4u | Error: %4d | Correction: %6.2f | Speeds: L=%.2f R=%.2f | Custom params: baseL=%.1f baseR=%.1f maxCorr=%.1f\n", 
               ir_value, SETPOINT - ir_value, correction, left_speed, right_speed, 
               base_left_speed, base_right_speed, max_correction);
        last_print = current_time;
    }
    
    // Set motor speeds
    drive_signed(left_speed, right_speed);
}

// ==============================
// TEST FUNCTION
// ==============================

void line_follow_test(void) {
    printf("\n=== PID LINE FOLLOWING TEST ===\n");
    printf("Initializing motors and sensors...\n");
    
    // Initialize motors and IR sensor
    motor_encoder_init();
    ir_init(NULL);
    
    printf("PID Constants: Kp=%.3f, Kd=%.3f\n", KP, KD);
    printf("Base Speed: %.1f%%, Setpoint: %d\n", BASE_SPEED_LEFT, SETPOINT);
    printf("IR Range: 0-4095 (0=black, 4095=white)\n");
    printf("Starting PID line following...\n\n");
    
    // Initialize previous_time
    previous_time = time_us_32();
    
    while (true) {
        // follow_line_simple();
        follow_line_simple_with_params(26.75, 25, 10.5);
        sleep_ms(50); // 40Hz update rate
    }
}

// ==============================
// MAIN FUNCTION
// ==============================

// int main(void) {
//     stdio_init_all();
//     sleep_ms(4000);
    
//     printf("\n=== PID Line Following Robot ===\n");
//     line_follow_test();
    
//     return 0;
// }