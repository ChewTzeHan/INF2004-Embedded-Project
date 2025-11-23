#ifndef IMU_MOVEMENT_H
#define IMU_MOVEMENT_H

#include <stdint.h>
#include <stdbool.h>

// IMU Data Structure
typedef struct {
    float pitch;        // Pitch angle in degrees
    float roll;         // Roll angle in degrees  
    float yaw;          // Yaw/heading angle in degrees
    float accel_x;      // Accelerometer X (g)
    float accel_y;      // Accelerometer Y (g)
    float accel_z;      // Accelerometer Z (g)
    float gyro_x;       // Gyroscope X (dps) - if available
    float gyro_y;       // Gyroscope Y (dps) - if available
    float gyro_z;       // Gyroscope Z (dps) - if available
    float mag_x;        // Magnetometer X
    float mag_y;        // Magnetometer Y
    float mag_z;        // Magnetometer Z
} imu_data_t;

// PID Configuration Structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain  
    float kd;           // Derivative gain
    float setpoint;     // Target yaw angle
    float base_speed_left;   // Base speed for left motor
    float base_speed_right;  // Base speed for right motor
    float max_output;   // Maximum PID output
    float integral_max; // Maximum integral windup
} pid_config_t;

// PID State Structure
typedef struct {
    float prev_error;   // Previous error for derivative
    float integral;     // Integral accumulator
    bool enabled;       // PID enabled flag
} pid_state_t;

// Function declarations
// Initialize IMU, filters and PID structures for movement control.
void imu_movement_init(void);

// Read the latest IMU orientation and acceleration into the struct.
// Returns true if new data was read successfully.
bool read_imu_data(imu_data_t *data);

// Print IMU data over serial (debug use only).
void print_imu_data(const imu_data_t *data);

// Initialize PID configuration and state structures to default values.
void pid_controller_init(pid_config_t *config, pid_state_t *state);

// Standard PID update: compute correction based on current yaw and internal state.
float pid_controller_update(float current_yaw,
                            pid_config_t *config,
                            pid_state_t *state);

// PID update with progressive / adaptive gains for smoother turning.
float progressive_pid_controller_update(float current_yaw,
                                        pid_config_t *config,
                                        pid_state_t *state);

// Compute PID output and map it to left/right motor speeds to drive toward the setpoint.
void drive_to_yaw_setpoint(float current_yaw,
                           pid_config_t *config,
                           pid_state_t *state,
                           float *pid_output,
                           float *left_speed,
                           float *right_speed);

// Accessors for the global PID configuration and state used by the movement task.
pid_config_t *get_pid_config(void);
pid_state_t  *get_pid_state(void);

// Detect a stable initial yaw setpoint by averaging a number of samples.
// Returns true once a reliable setpoint has been established.
bool detect_initial_setpoint(int samples);

// Manually set the yaw target (in degrees) for the PID controller.
void set_yaw_setpoint(float target_yaw);

// FreeRTOS task that continuously updates IMU-based movement and PID control.
void imu_movement_task(void *params);

// Speed and distance functions (computed using IMU + encoders)
void imu_speed_calc_init(void);       // Reset timers and distance accumulators
void imu_update_speed_and_distance(void);
float imu_get_current_speed_cm_s(void);
float imu_get_total_distance_cm(void);
void imu_reset_total_distance(void);

#endif // IMU_MOVEMENT_H