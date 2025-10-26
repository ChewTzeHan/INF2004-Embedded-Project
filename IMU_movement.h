#ifndef IMU_MOVEMENT_H
#define IMU_MOVEMENT_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_encoder_demo.h"
#include "message_buffer.h"

// PID configuration structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain  
    float kd;           // Derivative gain
    float setpoint;     // Target magnetometer X value
    int base_speed_left;     // Base motor speed (0-100)
    int base_speed_right;    // Base motor speed (0-100)
    int max_output;     // Maximum PID output limit
    float integral_max; // Anti-windup limit for integral term
} pid_config_t;

// PID state structure
typedef struct {
    float prev_error;
    float integral;
    bool enabled;
} pid_state_t;

// IMU data structure
typedef struct {
    float pitch;
    float roll;
    float yaw;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;    // Add magnetometer X
    float mag_y;    // Add magnetometer Y
    float mag_z;    // Add magnetometer Z
} imu_data_t;

// Function declarations
void imu_movement_init(void);
bool read_imu_data(imu_data_t *data);
void print_imu_data(const imu_data_t *data);
void imu_movement_task(void *params);

// PID controller functions
void pid_controller_init(pid_config_t *config, pid_state_t *state);
float pid_controller_update(float current_value, pid_config_t *config, pid_state_t *state);
void drive_to_mag_x_setpoint(float current_mag_x, pid_config_t *config, pid_state_t *state);

// Getter functions for PID configuration
pid_config_t* get_pid_config(void);
pid_state_t* get_pid_state(void);

#endif // IMU_MOVEMENT_H