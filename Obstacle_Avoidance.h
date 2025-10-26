#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdbool.h>

// ==============================
// OBSTACLE AVOIDANCE CONFIGURATION
// ==============================

// Distances
#define INITIAL_STOP_DISTANCE_CM       25.0f    // Stop when first detecting object
#define OBSTACLE_DETECTION_DISTANCE_CM 50.0f    // General detection range
#define SAFE_DISTANCE_CM               15.0f    // Too close distance

// Servo configuration
#define SERVO_PIN 15
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500

#define ENC1_DIG 6   // left / motor1 digital encoder
#define ENC2_DIG 4   // right / motor2 digital encoder
// Motor speeds
#define BASE_SPEED_LEFT 30.0f
#define BASE_SPEED_RIGHT 22.0f
#define CIRCLE_BASE_SPEED_LEFT 30.0f
#define CIRCLE_BASE_SPEED_RIGHT 22.0f

// PID parameters
#define PID_KP 2.0f
#define PID_KI 0.1f
#define PID_KD 0.5f
#define PID_TARGET_DISTANCE_CM 15.0f
#define PID_MAX_CORRECTION 20.0f

// Object scanning structure
typedef struct {
    float left_distance;
    float right_distance;
    float left_angle;
    float right_angle;
    float width_cm;
} ObjectScanResult;

// Function declarations
void servo_init(void);
void servo_set_angle(float angle);
bool obstacle_detected(float distance_cm);
bool object_too_close(float distance_cm);
bool object_at_stop_distance(float distance_cm);
void print_obstacle_status(float distance_cm);
float calculate_object_width(float distance_left, float distance_right, float servo_angle_left, float servo_angle_right);
ObjectScanResult scan_object_width(void);
void turn_right_on_spot(void);
float pid_calculate_correction(float current_distance);
void encircle_object_with_pid(void);
void encircle_object_simple(void);
void complete_avoidance_cycle(void);
void avoid_obstacle_simple(void);
void obstacle_avoidance_test(void);

#endif // OBSTACLE_AVOIDANCE_H