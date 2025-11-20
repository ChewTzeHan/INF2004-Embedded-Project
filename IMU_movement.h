/**
 * IMU_movement.h - Header file for IMU motion tracking and sensor data structures.
 */

#ifndef IMU_MOVEMENT_H
#define IMU_MOVEMENT_H

#include <stdbool.h>
#include <stdint.h>

// IMU Data Structure
typedef struct {
    float pitch;   // Pitch angle in degrees
    float roll;    // Roll angle in degrees
    float yaw;     // Yaw/heading angle in degrees
    float accel_x; // Accelerometer X (g)
    float accel_y; // Accelerometer Y (g)
    float accel_z; // Accelerometer Z (g)
    float gyro_x;  // Gyroscope X (dps) - if available
    float gyro_y;  // Gyroscope Y (dps) - if available
    float gyro_z;  // Gyroscope Z (dps) - if available
    float mag_x;   // Magnetometer X
    float mag_y;   // Magnetometer Y
    float mag_z;   // Magnetometer Z
} imu_data_t;

// PID Configuration Structure
typedef struct {
    float kp;               // Proportional gain
    float ki;               // Integral gain
    float kd;               // Derivative gain
    float setpoint;         // Target yaw angle
    float base_speed_left;  // Base speed for left motor
    float base_speed_right; // Base speed for right motor
    float max_output;       // Maximum PID output
    float integral_max;     // Maximum integral windup
} pid_config_t;

// PID State Structure
typedef struct {
    float prev_error; // Previous error for derivative
    float integral;   // Integral accumulator
    bool  enabled;    // PID enabled flag
} pid_state_t;

// Function declarations
void          imu_movement_init(void);
bool          read_imu_data(imu_data_t* data);
void          print_imu_data(const imu_data_t* data);
void          pid_controller_init(pid_config_t* config, pid_state_t* state);
float         pid_controller_update(float current_yaw, pid_config_t* config, pid_state_t* state);
float         progressive_pid_controller_update(float current_yaw, pid_config_t* config,
                                                pid_state_t* state);
void          drive_to_yaw_setpoint(float current_yaw, pid_config_t* config, pid_state_t* state,
                                    float* pid_output, float* left_speed, float* right_speed);
pid_config_t* get_pid_config(void);
pid_state_t*  get_pid_state(void);
bool          detect_initial_setpoint(int samples);
void          set_yaw_setpoint(float target_yaw);
void          imu_movement_task(void* params);

// Speed and distance functions (same pattern as obstacle avoidance)
void  imu_speed_calc_init(void);
void  imu_update_speed_and_distance(void);
float imu_get_current_speed_cm_s(void);
float imu_get_total_distance_cm(void);
void  imu_reset_total_distance(void);

#endif // IMU_MOVEMENT_H
