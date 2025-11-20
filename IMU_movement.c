/**
 * IMU_movement.c - IMU sensor integration with motion tracking and orientation calculations.
 */

#include "IMU_movement.h"
#include "FreeRTOS.h"
#include "encoder.h" // For speed calculation
#include "hardware/i2c.h"
#include "imu_raw_demo.h"       // Reuse existing IMU functions
#include "motor_encoder_demo.h" // For drive_signed
#include "mqtt_client.h"        // Include MQTT header
#include "pico/stdlib.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// Global variables for filtering
#define FILTER_SIZE 10
static int     filter_index            = 0;
static int16_t ax_history[FILTER_SIZE] = {0};
static int16_t ay_history[FILTER_SIZE] = {0};
static int16_t az_history[FILTER_SIZE] = {0};

// Global PID configuration and state - UPDATED FOR STABILITY
static pid_config_t pid_config = {
    .kp               = 0.38f,  // Increased from 0.08f - stronger correction
    .ki               = 0.005f, // Increased from 0.001f - some integral help
    .kd               = 0.20f,  // Increased from 0.15f - more damping
    .setpoint         = 140.0f,
    .base_speed_left  = 30.0f * 1.25,
    .base_speed_right = 30.0f,
    .max_output       = 15.0f, // Increased from 3.0f - much more authority
    .integral_max     = 30.0f  // Increased from 10.0f
};

static pid_state_t pid_state = {.prev_error = 0.0f, .integral = 0.0f, .enabled = false};

// ==============================
// SPEED CALCULATION VARIABLES (Same pattern as obstacle avoidance)
// ==============================

static volatile uint32_t g_imu_last_speed_calc_time = 0;
static volatile float    g_imu_current_speed_cm_s   = 0.0f;
static volatile float    g_imu_total_distance_cm    = 0.0f;
static volatile float    g_imu_last_distance_cm     = 0.0f;
static volatile uint32_t g_imu_last_pub_ms          = 0;

// ==============================
// SPEED CALCULATION FUNCTIONS (Same pattern as obstacle avoidance)
// ==============================

void imu_speed_calc_init(void)
{
    // Use the same interrupt-based encoder system as obstacle avoidance
    encoders_init(true); // Initialize encoders with pull-up

    g_imu_last_speed_calc_time = to_ms_since_boot(get_absolute_time());
    g_imu_current_speed_cm_s   = 0.0f;
    g_imu_total_distance_cm    = 0.0f;
    g_imu_last_distance_cm     = 0.0f;
    printf("[IMU_SPEED_CALC] Initialized with interrupt-based encoders\n");
}

void imu_update_speed_and_distance(void)
{
    uint32_t current_time    = to_ms_since_boot(get_absolute_time());
    uint32_t time_elapsed_ms = current_time - g_imu_last_speed_calc_time;

    if (time_elapsed_ms < 100)
        return; // Wait for at least 100ms for meaningful speed calculation

    // Get current distance using interrupt-based functions
    float left_distance    = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance   = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float current_distance = (left_distance + right_distance) / 2.0f;

    // Calculate distance traveled since last measurement
    float distance_traveled = current_distance - g_imu_last_distance_cm;

    // Calculate speed (cm/s) = distance (cm) / time (s)
    float time_elapsed_s = time_elapsed_ms / 1000.0f;

    if (time_elapsed_s > 0.0f) {
        g_imu_current_speed_cm_s = distance_traveled / time_elapsed_s;

        // Add sanity checks for reasonable speed values
        if (g_imu_current_speed_cm_s < 0) {
            g_imu_current_speed_cm_s = 0.0f; // No negative speeds
        }
        if (g_imu_current_speed_cm_s > 200.0f) { // Max reasonable speed for small robot
            g_imu_current_speed_cm_s = 200.0f;
        }
    } else {
        g_imu_current_speed_cm_s = 0.0f;
    }

    // Update total distance
    g_imu_total_distance_cm = current_distance;

    // Debug output (optional - can be commented out)
    printf("[IMU_SPEED] dist_traveled=%.2fcm, time=%.3fs, speed=%.2fcm/s\n", distance_traveled,
           time_elapsed_s, g_imu_current_speed_cm_s);

    // Update for next calculation
    g_imu_last_distance_cm     = current_distance;
    g_imu_last_speed_calc_time = current_time;
}

float imu_get_current_speed_cm_s(void)
{
    return g_imu_current_speed_cm_s;
}

float imu_get_total_distance_cm(void)
{
    // Use the interrupt-based distance calculation
    float left_distance  = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    return (left_distance + right_distance) / 2.0f;
}

void imu_reset_total_distance(void)
{
    // Use the interrupt-based reset function
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    g_imu_total_distance_cm = 0.0f;
    printf("[IMU_SPEED_CALC] Total distance reset\n");
}

// ==============================
// STATIC HELPER FUNCTIONS (unchanged)
// ==============================

static int16_t apply_filter(int16_t raw_value, int16_t history[], int* index)
{
    history[*index] = raw_value;
    *index          = (*index + 1) % FILTER_SIZE;

    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += history[i];
    }
    return (int16_t)(sum / FILTER_SIZE);
}

static void calculate_orientation(int16_t ax, int16_t ay, int16_t az, float* pitch, float* roll)
{
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / (float)M_PI;
    *roll  = atan2f(ay, az) * 180.0f / (float)M_PI;
}

static float calculate_yaw(int16_t mx, int16_t my, int16_t mz, float pitch_deg, float roll_deg)
{
    float pitch = pitch_deg * M_PI / 180.0f;
    float roll  = roll_deg * M_PI / 180.0f;

    // Tilt compensation
    float Mx_comp = mx * cosf(pitch) + mz * sinf(pitch);
    float My_comp = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

    float yaw = atan2f(-My_comp, Mx_comp) * 180.0f / M_PI;
    if (yaw < 0)
        yaw += 360.0f;

    return yaw;
}

static float normalize_angle(float angle)
{
    while (angle < 0)
        angle += 360.0f;
    while (angle >= 360.0f)
        angle -= 360.0f;
    return angle;
}

static float calculate_angle_error(float current_angle, float target_angle)
{
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
// TELEMETRY PUBLISHING FUNCTION (Updated with speed/distance)
// ==============================

// ==============================
// TELEMETRY PUBLISHING FUNCTION (Updated with speed/distance and accelerometer data)
// ==============================
static void publish_imu_telemetry(const imu_data_t* data, float error, float pid_output,
                                  float left_speed, float right_speed, int16_t ax_raw,
                                  int16_t ay_raw, int16_t az_raw, int16_t ax_filtered,
                                  int16_t ay_filtered, int16_t az_filtered)
{
    if (!mqtt_is_connected()) {
        return;
    }

    // Get speed and distance using the same pattern as obstacle avoidance
    float speed       = imu_get_current_speed_cm_s(); // Current speed in cm/s
    float distance_cm = imu_get_total_distance_cm();  // Total distance traveled
    float yaw_deg     = data->yaw;                    // Current yaw angle
    float ultra_cm    = 0.0f;                         // Not used in IMU mode

    // Create state string with PID values, SETPOINT, and accelerometer data
    char state[256]; // Increased buffer size to accommodate additional data

    if (pid_state.enabled && fabsf(error) > 1.0f) {
        snprintf(state, sizeof(state),
                 "adjusting: setpoint=%.1f, error=%.1f, pid=%.1f, L=%.1f, R=%.1f | "
                 "Accel Raw(X:%d Y:%d Z:%d) Filtered(X:%d Y:%d Z:%d)",
                 pid_config.setpoint, error, pid_output, left_speed, right_speed, ax_raw, ay_raw,
                 az_raw, ax_filtered, ay_filtered, az_filtered);
    } else {
        snprintf(state, sizeof(state),
                 "idle: setpoint=%.1f, error=%.1f, pid=%.1f, L=%.1f, R=%.1f | "
                 "Accel Raw(X:%d Y:%d Z:%d) Filtered(X:%d Y:%d Z:%d)",
                 pid_config.setpoint, error, pid_output, left_speed, right_speed, ax_raw, ay_raw,
                 az_raw, ax_filtered, ay_filtered, az_filtered);
    }

    // Publish telemetry via MQTT (same function call as obstacle avoidance)
    mqtt_publish_telemetry(speed, distance_cm, yaw_deg, ultra_cm, state);

    printf("[IMU_MQTT] Published - Speed: %.2f cm/s, Distance: %.2f cm, Yaw: %.1f°, State: %s\n",
           speed, distance_cm, yaw_deg, state);
}

// ==============================
// INITIALIZATION FUNCTIONS (Updated with speed init)
// ==============================

void imu_movement_init(void)
{
    printf("IMU Movement System Initializing...\n");

    // Initialize the existing IMU system
    imu_init();

    // Initialize speed calculation system
    imu_speed_calc_init();

    printf("IMU Movement System Ready\n");
    printf("• Reading pitch, roll, yaw from accelerometer and magnetometer\n");
    printf("• Using same I2C configuration as existing IMU system\n");
    printf("• Data filtering applied for stable readings\n");
    printf("• PID control using YAW angle for heading control\n");
    printf("• Speed and distance tracking via encoders\n");
    printf("• MQTT telemetry publishing enabled for yaw/heading, PID data, speed and distance\n\n");
}

// ==============================
// MAIN IMU READING FUNCTION (unchanged)
// ==============================

bool read_imu_data(imu_data_t* data)
{
    if (data == NULL)
        return false;

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
    data->roll  = normalize_angle(roll);
    data->yaw   = normalize_angle(yaw);

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
// DATA PRINTING FUNCTION (unchanged)
// ==============================

void print_imu_data(const imu_data_t* data)
{
    if (data == NULL)
        return;

    printf("[IMU] ");
    printf("Pitch:%6.1f° ", data->pitch);
    printf("Roll:%6.1f° ", data->roll);
    printf("Yaw:%6.1f° ", data->yaw);
    printf("\n");
    printf("Accel(X:%5.2fg Y:%5.2fg Z:%5.2fg)", data->accel_x, data->accel_y, data->accel_z);
    printf("\n");
    // Only print gyro if available
    if (data->gyro_x != 0 || data->gyro_y != 0 || data->gyro_z != 0) {
        printf(" Gyro(X:%5.1f Y:%5.1f Z:%5.1f)", data->gyro_x, data->gyro_y, data->gyro_z);
    }

    printf("\n");
}

// ==============================
// PID CONTROLLER FUNCTIONS (unchanged)
// ==============================

void pid_controller_init(pid_config_t* config, pid_state_t* state)
{
    if (config == NULL) {
        config = &pid_config;
    }

    if (state == NULL) {
        state = &pid_state;
    }

    state->prev_error = 0.0f;
    state->integral   = 0.0f;
    state->enabled    = true;

    printf("PID Controller Initialized\n");
    printf("KP: %.3f, KI: %.3f, KD: %.3f\n", config->kp, config->ki, config->kd);
    printf("Setpoint: %.1f°, Base Speed Left: %.1f, Base Speed Right: %.1f, Max Output: %.1f\n",
           config->setpoint, config->base_speed_left, config->base_speed_right, config->max_output);
    printf("Using YAW angle for heading control\n");
}

float progressive_pid_controller_update(float current_yaw, pid_config_t* config, pid_state_t* state)
{
    if (!state->enabled || config == NULL) {
        return 0.0f;
    }

    float error = calculate_angle_error(current_yaw, config->setpoint);

    // Progressive gains - MORE AGGRESSIVE for large errors
    float effective_kp, effective_kd;

    if (fabsf(error) < 10.0f) {
        // Small errors: gentle correction
        effective_kp = 0.30f; // Increased
        effective_kd = 0.25f; // Increased
    } else if (fabsf(error) < 25.0f) {
        // Medium errors: moderate correction
        effective_kp = 0.20f; // Increased
        effective_kd = 0.15f;
    } else if (fabsf(error) < 45.0f) {
        // Large errors: strong correction
        effective_kp = 0.15f;
        effective_kd = 0.10f;
    } else {
        // Very large errors: maximum correction to recover
        effective_kp = 0.12f;
        effective_kd = 0.08f;
    }

    // Proportional term
    float proportional = effective_kp * error;

    // Integral term - reset on very large errors to prevent windup during recovery
    if (fabsf(error) < 45.0f) {
        state->integral += error;
    } else {
        state->integral = 0.0f; // Reset integral during large corrections
    }

    if (state->integral > config->integral_max)
        state->integral = config->integral_max;
    if (state->integral < -config->integral_max)
        state->integral = -config->integral_max;
    float integral = config->ki * state->integral;

    // Derivative term
    float derivative  = effective_kd * (error - state->prev_error);
    state->prev_error = error;

    // Calculate PID output
    float output = proportional + integral + derivative;

    // Clamp output
    if (output > config->max_output)
        output = config->max_output;
    if (output < -config->max_output)
        output = -config->max_output;

    return output;
}

float pid_controller_update(float current_yaw, pid_config_t* config, pid_state_t* state)
{
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
    float derivative  = config->kd * (error - state->prev_error);
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

void drive_to_yaw_setpoint(float current_yaw, pid_config_t* config, pid_state_t* state,
                           float* pid_output, float* left_speed, float* right_speed)
{
    if (!state->enabled) {
        // Stop motors if PID is disabled
        *pid_output  = 0.0f;
        *left_speed  = 0.0f;
        *right_speed = 0.0f;
        return;
    }

    // Get PID output
    *pid_output = progressive_pid_controller_update(current_yaw, config, state);

    // Calculate motor speeds
    // Positive output means turn right, negative means turn left
    *left_speed  = config->base_speed_left + *pid_output;  // Add for left motor
    *right_speed = config->base_speed_right - *pid_output; // Subtract for right motor

    // Clamp motor speeds to valid range (-100 to 100)
    if (*left_speed > 100.0f)
        *left_speed = 100.0f;
    if (*left_speed < -100.0f)
        *left_speed = -100.0f;
    if (*right_speed > 100.0f)
        *right_speed = 100.0f;
    if (*right_speed < -100.0f)
        *right_speed = -100.0f;

    // Apply motor control
    printf("THE SPEEDS ARRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRE: L=%.1f R=%.1f\n",
           *left_speed, *right_speed);
    drive_signed(*left_speed, *right_speed);
}

// ==============================
// GETTER FUNCTIONS FOR PID CONFIG (unchanged)
// ==============================

pid_config_t* get_pid_config(void)
{
    return &pid_config;
}

pid_state_t* get_pid_state(void)
{
    return &pid_state;
}

// ==============================
// AUTO-SETPOINT DETECTION FUNCTION (unchanged)
// ==============================

bool detect_initial_setpoint(int samples)
{
    printf("Detecting initial heading (yaw)...\n");
    printf("Please keep the robot stationary for %d samples...\n", samples);

    float sum_yaw            = 0.0f;
    int   successful_samples = 0;

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
        float average_yaw   = sum_yaw / successful_samples;
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
// SET YAW SETPOINT FUNCTION (unchanged)
// ==============================

void set_yaw_setpoint(float target_yaw)
{
    pid_config.setpoint = normalize_angle(target_yaw);
    printf("Yaw setpoint updated to: %.1f°\n", pid_config.setpoint);
}

// ==============================
// FREE RTOS TASK FUNCTION (Updated with speed calculation)
// ==============================
void imu_movement_task(__unused void* params)
{
    printf("IMU Movement Task Started\n");

    // Initialize IMU
    imu_movement_init();

    imu_data_t     current_data;
    uint32_t       last_print_time   = 0;
    const uint32_t print_interval_ms = 30; // Print every 30ms

    // Add local variables to store raw accelerometer data
    int16_t ax_raw, ay_raw, az_raw;
    int16_t ax_filtered, ay_filtered, az_filtered;

    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Read IMU data
        if (read_imu_data(&current_data)) {
            // Read raw accelerometer data separately for telemetry
            if (read_accel_raw(&ax_raw, &ay_raw, &az_raw)) {
                // Apply filtering to get filtered values
                ax_filtered = apply_filter(ax_raw, ax_history, &filter_index);
                ay_filtered = apply_filter(ay_raw, ay_history, &filter_index);
                az_filtered = apply_filter(az_raw, az_history, &filter_index);
            }

            float pid_output, left_speed, right_speed;

            // Apply PID control for yaw
            drive_to_yaw_setpoint(current_data.yaw, &pid_config, &pid_state, &pid_output,
                                  &left_speed, &right_speed);

            float error = calculate_angle_error(current_data.yaw, pid_config.setpoint);

            // Update speed and distance calculation
            imu_update_speed_and_distance();

            // Publish MQTT telemetry with ALL parameters including accelerometer data
            if (mqtt_is_connected() && (current_time - g_imu_last_pub_ms >= 500)) {
                g_imu_last_pub_ms = current_time;
                publish_imu_telemetry(&current_data, error, pid_output, left_speed, right_speed,
                                      ax_raw, ay_raw, az_raw, ax_filtered, ay_filtered,
                                      az_filtered);
            }

            // Print data at specified interval
            if (current_time - last_print_time >= print_interval_ms) {
                float current_speed  = imu_get_current_speed_cm_s();
                float total_distance = imu_get_total_distance_cm();

                printf("[IMU] Yaw: %.1f° | Setpoint: %.1f° | Error: %.1f° | PID: %.1f | L: %.1f | "
                       "R: %.1f | Speed: %.1f cm/s | Dist: %.1f cm\n",
                       current_data.yaw, pid_config.setpoint, error, pid_output, left_speed,
                       right_speed, current_speed, total_distance);
                last_print_time = current_time;
            }
        } else {
            printf("[IMU] Failed to read sensor data\n");
        }

        // Handle MQTT polling
        mqtt_loop_poll();

        // Task delay
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==============================
// STANDALONE TEST MAIN FUNCTION (Updated with speed init)
// ==============================

static void imu_robot_task(void* pv)
{
    // optional: small delay to let USB come up
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("\n\n=== IMU Movement System with YAW PID Control and MQTT ===\n");
    printf("Initializing IMU...\n");

    if (!wifi_and_mqtt_start()) {
        printf("[NET] Wi-Fi/MQTT start failed (continuing without MQTT)\n");
    }

    // Initialize the IMU movement system (includes speed calculation)
    imu_movement_init();

    // Initialize motors
    motor_encoder_init();

    // Detect initial setpoint automatically
    printf("\n=== Auto-Setpoint Detection ===\n");
    if (!detect_initial_setpoint(10)) { // Take 10 samples for averaging
        printf("Falling back to manual setpoint: 0.0°\n");
        pid_config.setpoint = 140.0f; // Fallback value
    }

    // Initialize PID controller with detected setpoint
    pid_controller_init(NULL, NULL);

    printf("\nStarting IMU data reading loop with PID control and MQTT telemetry...\n");
    printf("Format: Yaw: [angle]° | Setpoint: [angle]° | Error: [error]° | PID: [output] | L: "
           "[left_speed] | R: [right_speed] | Speed: [speed] cm/s | Dist: [distance] cm\n\n");

    imu_data_t     imu_data;
    uint32_t       last_print_time          = 0;
    uint32_t       last_pid_time            = 0;
    uint32_t       last_speed_update_time   = 0;
    const uint32_t pid_interval_ms          = 50;  // 20Hz PID control
    const uint32_t speed_update_interval_ms = 100; // 10Hz speed update
    int16_t        ax_raw, ay_raw, az_raw;
    int16_t        ax_filtered, ay_filtered, az_filtered;
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        // Read IMU data
        if (read_imu_data(&imu_data)) {
            float pid_output, left_speed, right_speed;
            float error = calculate_angle_error(imu_data.yaw, pid_config.setpoint);
            if (read_accel_raw(&ax_raw, &ay_raw, &az_raw)) {
                // Apply filtering
                ax_filtered = apply_filter(ax_raw, ax_history, &filter_index);
                ay_filtered = apply_filter(ay_raw, ay_history, &filter_index);
                az_filtered = apply_filter(az_raw, az_history, &filter_index);
            }

            // Apply PID control at higher frequency (20Hz)
            if (current_time - last_pid_time >= pid_interval_ms) {
                drive_to_yaw_setpoint(imu_data.yaw, &pid_config, &pid_state, &pid_output,
                                      &left_speed, &right_speed);
                last_pid_time = current_time;
            }

            // Update speed calculation at moderate frequency (10Hz)
            if (current_time - last_speed_update_time >= speed_update_interval_ms) {
                imu_update_speed_and_distance();
                last_speed_update_time = current_time;
            }

            // Publish MQTT telemetry with PID values and speed/distance
            if (mqtt_is_connected() && (current_time - g_imu_last_pub_ms >= 500)) {
                g_imu_last_pub_ms = current_time;
                publish_imu_telemetry(&imu_data, error, pid_output, left_speed, right_speed, ax_raw,
                                      ay_raw, az_raw, ax_filtered, ay_filtered, az_filtered);
            }

            // Print data every 500ms with comprehensive information
            if (current_time - last_print_time >= 500) {
                float current_speed  = imu_get_current_speed_cm_s();
                float total_distance = imu_get_total_distance_cm();

                printf("Yaw: %6.1f° | Setpoint: %6.1f° | Error: %6.1f° | PID: %6.1f | L: %6.1f | "
                       "R: %6.1f | Speed: %6.1f cm/s | Dist: %6.1f cm\n",
                       imu_data.yaw, pid_config.setpoint, error, pid_output, left_speed,
                       right_speed, current_speed, total_distance);
                last_print_time = current_time;
                // pid_config.setpoint += 0.4f; // Increment setpoint for testing
            }
        } else {
            printf("[ERROR] Failed to read IMU data - Stopping motors\n");
            // Stop motors on sensor failure
        }

        // Handle MQTT polling (same as obstacle avoidance)
        mqtt_loop_poll();

        // Use vTaskDelay for FreeRTOS tasks
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz loop for responsive control
    }

    vTaskDelete(NULL); // never reached, but good practice
}

// int main(void) {
//     stdio_init_all();

//     // Wait for serial connection to be established
//     sleep_ms(4000);

//     // Create the IMU robot task
//     xTaskCreate(
//         imu_robot_task,
//         "imu_robot",
//         4096,                  // stack words
//         NULL,
//         tskIDLE_PRIORITY + 2,  // priority
//         NULL
//     );

//     // Start FreeRTOS
//     vTaskStartScheduler();

//     // Should never return
//     while (1) { /* idle */ }
// }
