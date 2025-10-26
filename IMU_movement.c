#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "IMU_movement.h"
#include "imu_raw_demo.h"  // Reuse existing IMU functions

// Global variables for filtering
#define FILTER_SIZE 10
static int filter_index = 0;
static int16_t ax_history[FILTER_SIZE] = {0};
static int16_t ay_history[FILTER_SIZE] = {0};
static int16_t az_history[FILTER_SIZE] = {0};

// Global PID configuration and state
static pid_config_t pid_config = {
    .kp = 0.2f,           // Increased proportional gain for angle control
    .ki = 0.03f,          // Integral gain
    .kd = 0.05f,          // Derivative gain for damping
    .setpoint = 0.0f,     // Will be set to initial yaw value
    .base_speed_left = 32.5f,     // Base motor speed (0-100)
    .base_speed_right = 30.0f,    // Base motor speed (0-100)
    .max_output = 20.0f,  // Maximum PID output for angle control
    .integral_max = 200.0f // Anti-windup limit
};

static pid_state_t pid_state = {
    .prev_error = 0.0f,
    .integral = 0.0f,
    .enabled = false
};

// ==============================
// STATIC HELPER FUNCTIONS
// ==============================

/**
 * @brief Apply simple moving average filter to raw sensor data
 */
static int16_t apply_filter(int16_t raw_value, int16_t history[], int *index) {
    history[*index] = raw_value;
    *index = (*index + 1) % FILTER_SIZE;

    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += history[i];
    }
    return (int16_t)(sum / FILTER_SIZE);
}

/**
 * @brief Calculate orientation angles from accelerometer data
 */
static void calculate_orientation(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll) {
    // Convert raw accelerometer data to orientation angles
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;
    *roll = atan2f(ay, az) * 180.0f / (float)M_PI;
}

/**
 * @brief Calculate yaw angle from magnetometer data with tilt compensation
 */
static float calculate_yaw(int16_t mx, int16_t my, int16_t mz, float pitch_deg, float roll_deg) {
    float pitch = pitch_deg * M_PI / 180.0f;
    float roll  = roll_deg  * M_PI / 180.0f;

    // Tilt compensation
    float Mx_comp = mx * cosf(pitch) + mz * sinf(pitch);
    float My_comp = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    float yaw = atan2f(-My_comp, Mx_comp) * 180.0f / M_PI;
    if (yaw < 0) yaw += 360.0f;

    return yaw;
}

/**
 * @brief Normalize angle to 0-360 degree range
 */
static float normalize_angle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

/**
 * @brief Calculate shortest path error between two angles (handles 0/360 wrap-around)
 */
static float calculate_angle_error(float current_angle, float target_angle) {
    float error = target_angle - current_angle;
    
    // Handle the 0/360 degree wrap-around
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    
    return error;
}

// ==============================
// INITIALIZATION FUNCTIONS
// ==============================

void imu_movement_init(void) {
    printf("IMU Movement System Initializing...\n");
    
    // Initialize the existing IMU system
    imu_init();
    
    printf("IMU Movement System Ready\n");
    printf("• Reading pitch, roll, yaw from accelerometer and magnetometer\n");
    printf("• Using same I2C configuration as existing IMU system\n");
    printf("• Data filtering applied for stable readings\n");
    printf("• PID control using YAW angle for heading control\n\n");
}

// ==============================
// MAIN IMU READING FUNCTION
// ==============================

bool read_imu_data(imu_data_t *data) {
    if (data == NULL) return false;
    
    // Initialize data structure
    memset(data, 0, sizeof(imu_data_t));
    
    // Read raw accelerometer data
    int16_t ax, ay, az;
    if (!read_accel_raw(&ax, &ay, &az)) {
        return false;
    }
    
    // Apply filtering to accelerometer data
    int16_t ax_filtered = apply_filter(ax, ax_history, &filter_index);
    int16_t ay_filtered = apply_filter(ay, ay_history, &filter_index);
    int16_t az_filtered = apply_filter(az, az_history, &filter_index);
    
    // Read magnetometer data for yaw (with Z axis)
    int16_t mx, my, mz;
    if (!read_mag_raw(&mx, &my, &mz)) {
        return false;
    }
    
    // Calculate orientation
    float pitch, roll;
    calculate_orientation(ax_filtered, ay_filtered, az_filtered, &pitch, &roll);
    
    // Use the new calculate_yaw function with tilt compensation
    float yaw = calculate_yaw(mx, my, mz, pitch, roll);
    
    // Normalize angles
    data->pitch = normalize_angle(pitch);
    data->roll = normalize_angle(roll);
    data->yaw = normalize_angle(yaw);
    
    // Store filtered accelerometer data
    data->accel_x = ax_filtered / 1000.0f;
    data->accel_y = ay_filtered / 1000.0f;
    data->accel_z = az_filtered / 1000.0f;
    
    // Store magnetometer data (raw values)
    data->mag_x = (float)mx;
    data->mag_y = (float)my;
    data->mag_z = (float)mz;
    
    // Note: The current IMU setup doesn't have gyroscope readings
    data->gyro_x = 0.0f;
    data->gyro_y = 0.0f;
    data->gyro_z = 0.0f;
    
    return true;
}

// ==============================
// DATA PRINTING FUNCTION
// ==============================

void print_imu_data(const imu_data_t *data) {
    if (data == NULL) return;
    
    printf("[IMU] ");
    printf("Pitch:%6.1f° ", data->pitch);
    printf("Roll:%6.1f° ", data->roll);
    printf("Yaw:%6.1f° ", data->yaw);
    printf("\n");
    printf("Accel(X:%5.2fg Y:%5.2fg Z:%5.2fg)", 
           data->accel_x, data->accel_y, data->accel_z);
    printf("\n");
    // Only print gyro if available
    if (data->gyro_x != 0 || data->gyro_y != 0 || data->gyro_z != 0) {
        printf(" Gyro(X:%5.1f Y:%5.1f Z:%5.1f)", 
               data->gyro_x, data->gyro_y, data->gyro_z);
    }
    
    printf("\n");
}

// ==============================
// PID CONTROLLER FUNCTIONS
// ==============================

void pid_controller_init(pid_config_t *config, pid_state_t *state) {
    if (config == NULL) {
        config = &pid_config;
    }
    
    if (state == NULL) {
        state = &pid_state;
    }
    
    state->prev_error = 0.0f;
    state->integral = 0.0f;
    state->enabled = true;
    
    printf("PID Controller Initialized\n");
    printf("KP: %.3f, KI: %.3f, KD: %.3f\n", config->kp, config->ki, config->kd);
    printf("Setpoint: %.1f°, Base Speed Left: %.1f, Base Speed Right: %.1f, Max Output: %.1f\n", 
           config->setpoint, config->base_speed_left, config->base_speed_right, config->max_output);
    printf("Using YAW angle for heading control\n");
}

float pid_controller_update(float current_yaw, pid_config_t *config, pid_state_t *state) {
    if (!state->enabled || config == NULL) {
        return 0.0f;
    }
    
    // Calculate error with proper angle wrapping
    float error = calculate_angle_error(current_yaw, config->setpoint);
    
    // Proportional term
    float proportional = config->kp * error;
    
    // Integral term with anti-windup
    state->integral += error;
    
    // Clamp integral term to prevent windup
    if (state->integral > config->integral_max) {
        state->integral = config->integral_max;
    } else if (state->integral < -config->integral_max) {
        state->integral = -config->integral_max;
    }
    
    float integral = config->ki * state->integral;
    
    // Derivative term
    float derivative = config->kd * (error - state->prev_error);
    state->prev_error = error;
    
    // Calculate PID output
    float output = proportional + integral + derivative;
    
    // Clamp output to maximum limits
    if (output > config->max_output) {
        output = config->max_output;
    } else if (output < -config->max_output) {
        output = -config->max_output;
    }
    
    return output;
}

void drive_to_yaw_setpoint(float current_yaw, pid_config_t *config, pid_state_t *state) {
    if (!state->enabled) {
        // Stop motors if PID is disabled
        drive_signed(0.0f, 0.0f);
        return;
    }
    
    // Get PID output
    float pid_output = pid_controller_update(current_yaw, config, state);
    
    // Calculate motor speeds
    // Positive output means turn right, negative means turn left
    float left_speed = config->base_speed_left + pid_output;  // Add for left motor
    float right_speed = config->base_speed_right - pid_output; // Subtract for right motor

    // Clamp motor speeds to valid range (-100 to 100)
    if (left_speed > 100.0f) left_speed = 100.0f;
    if (left_speed < -100.0f) left_speed = -100.0f;
    if (right_speed > 100.0f) right_speed = 100.0f;
    if (right_speed < -100.0f) right_speed = -100.0f;
    
    // Apply motor control
    drive_signed(left_speed, right_speed);
}

// ==============================
// GETTER FUNCTIONS FOR PID CONFIG
// ==============================

pid_config_t* get_pid_config(void) {
    return &pid_config;
}

pid_state_t* get_pid_state(void) {
    return &pid_state;
}

// ==============================
// AUTO-SETPOINT DETECTION FUNCTION
// ==============================

/**
 * @brief Detect and set the initial yaw value as setpoint
 * @param samples Number of samples to average for initial reading
 * @return true if successful, false if detection failed
 */
bool detect_initial_setpoint(int samples) {
    printf("Detecting initial heading (yaw)...\n");
    printf("Please keep the robot stationary for %d samples...\n", samples);
    
    float sum_yaw = 0.0f;
    int successful_samples = 0;
    
    for (int i = 0; i < samples; i++) {
        imu_data_t data;
        if (read_imu_data(&data)) {
            sum_yaw += data.yaw;
            successful_samples++;
            printf("Sample %d/%d: Yaw = %.1f°\n", i + 1, samples, data.yaw);
        } else {
            printf("Sample %d/%d: Failed to read IMU data\n", i + 1, samples);
        }
        sleep_ms(100); // 10Hz sampling for detection
    }
    
    if (successful_samples > 0) {
        float average_yaw = sum_yaw / successful_samples;
        pid_config.setpoint = average_yaw;
        
        printf("Initial setpoint detection complete!\n");
        printf("Successful samples: %d/%d\n", successful_samples, samples);
        printf("Average Yaw: %.1f° (set as setpoint)\n", average_yaw);
        return true;
    } else {
        printf("ERROR: Failed to detect initial setpoint - no successful readings\n");
        return false;
    }
}

// ==============================
// SET YAW SETPOINT FUNCTION
// ==============================

/**
 * @brief Set a specific yaw angle as the target setpoint
 * @param target_yaw The desired yaw angle in degrees (0-360)
 */
void set_yaw_setpoint(float target_yaw) {
    pid_config.setpoint = normalize_angle(target_yaw);
    printf("Yaw setpoint updated to: %.1f°\n", pid_config.setpoint);
}

// ==============================
// FREE RTOS TASK FUNCTION
// ==============================

void imu_movement_task(__unused void *params) {
    printf("IMU Movement Task Started\n");
    
    // Initialize IMU
    imu_movement_init();
    
    imu_data_t current_data;
    uint32_t last_print_time = 0;
    const uint32_t print_interval_ms = 500; // Print every 500ms
    
    while (true) {
        uint32_t current_time = time_us_32() / 1000;
        
        // Read IMU data
        if (read_imu_data(&current_data)) {
            // Apply PID control for yaw
            drive_to_yaw_setpoint(current_data.yaw, &pid_config, &pid_state);
            
            // Print data at specified interval
            if (current_time - last_print_time >= print_interval_ms) {
                float error = calculate_angle_error(current_data.yaw, pid_config.setpoint);
                float pid_output = pid_controller_update(current_data.yaw, &pid_config, &pid_state);
                
                printf("[IMU] Yaw: %.1f° | Setpoint: %.1f° | Error: %.1f° | PID: %.1f\n",
                       current_data.yaw, pid_config.setpoint, error, pid_output);
                last_print_time = current_time;
            }
        } else {
            printf("[IMU] Failed to read sensor data\n");
        }
        
        // Task delay - adjust based on your needs
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}

// ==============================
// STANDALONE TEST MAIN FUNCTION
// ==============================

int main(void) {
    stdio_init_all();
    
    // Wait for serial connection to be established
    sleep_ms(4000);
    
    printf("\n\n=== IMU Movement System with YAW PID Control ===\n");
    printf("Initializing IMU...\n");
    
    // Initialize the IMU movement system
    imu_movement_init();
    
    // Initialize motors
    motor_encoder_init();
    
    // Detect initial setpoint automatically
    printf("\n=== Auto-Setpoint Detection ===\n");
    if (!detect_initial_setpoint(10)) { // Take 10 samples for averaging
        printf("Falling back to manual setpoint: 0.0°\n");
        pid_config.setpoint = 0.0f; // Fallback value
    }
    
    // Initialize PID controller with detected setpoint
    pid_controller_init(NULL, NULL);
    
    printf("\nStarting IMU data reading loop with PID control...\n");
    printf("Format: Yaw: [angle]° | Setpoint: [angle]° | Error: [error]° | PID: [output] | L: [left_speed] | R: [right_speed]\n\n");
    
    imu_data_t imu_data;
    uint32_t last_print_time = 0;
    uint32_t last_pid_time = 0;
    const uint32_t pid_interval_ms = 50; // 20Hz PID control
    
    while (true) {
        uint32_t current_time = time_us_32() / 1000;
        
        // Read IMU data
        if (read_imu_data(&imu_data)) {
            // Apply PID control at higher frequency (20Hz)
            if (current_time - last_pid_time >= pid_interval_ms) {
                drive_to_yaw_setpoint(imu_data.yaw, &pid_config, &pid_state);
                last_pid_time = current_time;
            }
            
            // Print data every 500ms with PID information
            if (current_time - last_print_time >= 500) {
                // Get current PID values for display
                float error = calculate_angle_error(imu_data.yaw, pid_config.setpoint);
                float pid_output = pid_controller_update(imu_data.yaw, &pid_config, &pid_state);
                float left_speed = pid_config.base_speed_left + pid_output;
                float right_speed = pid_config.base_speed_right - pid_output;

                printf("Yaw: %6.1f° | Setpoint: %6.1f° | Error: %6.1f° | PID: %6.1f | L: %6.1f | R: %6.1f\n",
                       imu_data.yaw, pid_config.setpoint, error, pid_output, left_speed, right_speed);
                last_print_time = current_time;
            }
        } else {
            printf("[ERROR] Failed to read IMU data - Stopping motors\n");
            drive_signed(0.0f, 0.0f); // Stop motors on sensor failure
        }
        
        // Small delay to prevent overwhelming the system
        sleep_ms(10); // 100Hz loop for responsive control
    }
    
    return 0;
}