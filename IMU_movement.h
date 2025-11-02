#ifndef IMU_MOVEMENT_H
#define IMU_MOVEMENT_H

#include <stdbool.h>

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
    float mag_x;
    float mag_y;
    float mag_z;
} imu_data_t;

// PID configuration structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float base_speed_left;
    float base_speed_right;
    float max_output;
    float integral_max;
} pid_config_t;

// PID state structure
typedef struct {
    float prev_error;
    float integral;
    bool enabled;
} pid_state_t;

// Function declarations
void imu_movement_init(void);
bool read_imu_data(imu_data_t *data);
void print_imu_data(const imu_data_t *data);
void pid_controller_init(pid_config_t *config, pid_state_t *state);
void drive_with_encoder_balance(float *pid_output, float *left_speed, float *right_speed);
pid_config_t* get_pid_config(void);
pid_state_t* get_pid_state(void);
void set_pid_enabled(bool enabled);
void set_base_speed(float speed);

// FreeRTOS task
void imu_movement_task(__unused void *params);

#endif // IMU_MOVEMENT_H