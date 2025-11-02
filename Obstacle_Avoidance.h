#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdint.h>
#include <stdbool.h>

// Servo configuration
#define SERVO_PIN 15
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

// Motor speeds
#define BASE_SPEED_LEFT 32.75f
#define BASE_SPEED_RIGHT 30.0f
#define CIRCLE_BASE_SPEED_LEFT 42.25f
#define CIRCLE_BASE_SPEED_RIGHT 40.0f

// Distance thresholds (cm)
#define OBSTACLE_DETECTION_DISTANCE_CM 20.0f
#define SAFE_DISTANCE_CM 20.0f
#define INITIAL_STOP_DISTANCE_CM 20.0f

// Encoder pins
#define ENC1_DIG 10
#define ENC2_DIG 11

// Encoder turning configuration
#define TURN_90_DEG_PULSES 13
#define TURN_180_DEG_PULSES 100
#define TURN_45_DEG_PULSES 25

// External encoder functions from motor_encoder_demo.c
extern volatile uint32_t encoder_left_count;
extern volatile uint32_t encoder_right_count;
extern void update_encoder_counts(void);
extern void reset_encoder_counts(void);
extern uint32_t get_encoder_pulses(void);

// Object scanning structure
typedef struct {
    float left_distance;
    float right_distance;
    float left_angle;
    float right_angle;
    float width_cm;
} ObjectScanResult;

// Servo functions
void servo_init(void);
void servo_set_angle(float angle);

// Encoder functions
void update_encoder_counts(void);
void reset_encoder_counts(void);
uint32_t get_encoder_pulses(void);
void print_encoder_status(void);

// Obstacle detection
bool obstacle_detected(float distance_cm);
bool object_too_close(float distance_cm);
bool object_at_stop_distance(float distance_cm);

// Object scanning
float calculate_object_width(float distance_left, float distance_right, float servo_angle_left, float servo_angle_right);
ObjectScanResult scan_object_width(void);

// Turning functions
void turn_left_encoder_based(uint32_t target_pulses);
void turn_right_encoder_based(uint32_t target_pulses);
void turn_left_90_degrees(void);
void turn_right_90_degrees(void);
void turn_left_45_degrees(void);

// Circling functions
bool check_if_object_still_there(void);
void encircle_object_with_checking(void);
void encircle_object_simple(void);

// Main avoidance cycle
void complete_avoidance_cycle(void);

// Combined line following and obstacle avoidance
void line_follow_with_obstacle_avoidance(void);

// Test functions
void obstacle_avoidance_test(void);
void calibrate_turning_pulses(void);

// Add these function declarations to Obstacle_Avoidance.h
void follow_line_simple(void);
void set_motor_speeds_pid(uint16_t ir_value);
float pid_calculation(uint16_t ir_value);

#endif