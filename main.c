/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * Modified for WiFi Data Exchange Demo with Motor Control, Ultrasonic Sensor, IMU, IR Sensor, and Encoder
 * 
 * Description: Main application entry point
 */
#define PICO_MAX_SHARED_IRQ_HANDLERS 8
#include "imu_raw_demo.h"
#include "ir_sensor.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Pico-specific includes for WiFi and hardware
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// FreeRTOS includes for real-time operating system
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "queue.h"
#include "timers.h"

// Header file for server functions
#include "picow_freertos_ping.h"
#include "motor_encoder_demo.h"
#include "ultrasonic.h"

// Hardware peripheral includes
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#include <time.h>

// ==============================
// GLOBAL VARIABLES - Sensor Data Storage
// ==============================

// IR Sensor Data
typedef struct {
    uint16_t left_raw;
    bool right_digital;
    uint64_t last_change_time;
    bool last_digital_state;
} ir_data_t;

// Ultrasonic Data
typedef struct {
    float distance_cm;
    bool object_present;
} ultrasonic_data_t;

// IMU Data
typedef struct {
    float pitch;
    float roll;  
    float yaw;
} imu_data_t;

// Motor Data
typedef struct {
    int32_t encoder1_pulses;
    int32_t encoder2_pulses;
    uint32_t last_encoder_time;
} motor_data_t;

// Global sensor data structures
ir_data_t ir_sensor_data = {0};
ultrasonic_data_t ultrasonic_data = {0};
imu_data_t imu_data = {0};
motor_data_t motor_data = {0};

// Encoder state tracking
static bool last_enc1_state = false;
static bool last_enc2_state = false;

// Pico SDK repeating timers
struct repeating_timer imu_timer;
struct repeating_timer encoder_timer;
struct repeating_timer sensor_timer;
struct repeating_timer telemetry_timer;

// ==============================
// FUNCTION DECLARATIONS
// ==============================

// Forward declarations
float calculate_pid_output(uint16_t ir_raw, float yaw);
void set_motor_speed(float speed);
void avoid_obstacle(void);
void decode_barcode(void);

// Timer callback functions
bool read_imu_data_callback(struct repeating_timer *t);
bool update_encoders_callback(struct repeating_timer *t);
bool sensor_update_callback(struct repeating_timer *t);
bool telemetry_callback(struct repeating_timer *t);

// ==============================
// PICO SDK TIMER CALLBACKS
// ==============================

bool read_imu_data_callback(struct repeating_timer *t) {
    // IMU reading (20ms)
    int16_t ax, ay, az;
    if (read_accel_raw(&ax, &ay, &az)) {
        imu_data.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;
        imu_data.roll = atan2f(ay, az) * 180.0f / (float)M_PI;
        
        int16_t mx, my, mz;
        if (read_mag_raw(&mx, &my, &mz)) {
            // Use the new calculate_yaw function with tilt compensation
            // Note: You'll need to implement or call the calculate_yaw function from imu_raw_demo
            // For now, use a simple calculation or include the function
            float yaw_rad = atan2f(my, mx);
            imu_data.yaw = yaw_rad * 180.0f / (float)M_PI;
            if (imu_data.yaw < 0) imu_data.yaw += 360.0f;
        }
    }
    return true; // Continue repeating
}

bool update_encoders_callback(struct repeating_timer *t) {
    // Encoder reading (10ms)
    bool current_enc1 = gpio_get(6);
    bool current_enc2 = gpio_get(4);
    
    if (current_enc1 && !last_enc1_state) {
        motor_data.encoder1_pulses++;
    }
    if (current_enc2 && !last_enc2_state) {
        motor_data.encoder2_pulses++;
    }
    
    last_enc1_state = current_enc1;
    last_enc2_state = current_enc2;
    motor_data.last_encoder_time = time_us_32();
    return true; // Continue repeating
}

bool sensor_update_callback(struct repeating_timer *t) {
    // Combined sensor updates (50ms)
    // 1. Read IR sensors
    ir_sensor_data.left_raw = ir_read_raw();
    
    bool current_right_state = gpio_get(RIGHT_IR_DIGITAL_PIN);
    if (current_right_state != ir_sensor_data.last_digital_state) {
        ir_sensor_data.right_digital = current_right_state;
        ir_sensor_data.last_change_time = time_us_64();
        ir_sensor_data.last_digital_state = current_right_state;
        
        if (current_right_state) {
            printf("Right IR BLACK - barcode detected\n");
            decode_barcode();
        }
    }
    
    // 2. Read ultrasonic
    float distance = ultrasonic_get_distance_cm();
    ultrasonic_data.distance_cm = distance;
    ultrasonic_data.object_present = (distance > 2.0f && distance < 30.0f);
    
    if (ultrasonic_data.object_present) {
        printf("OBSTACLE at %.1fcm! Avoiding...\n", distance);
        avoid_obstacle();
    }
    
    // 3. Run PID control
    float motor_speed = calculate_pid_output(ir_sensor_data.left_raw, imu_data.yaw);
    set_motor_speed(motor_speed);
    
    return true; // Continue repeating
}

bool telemetry_callback(struct repeating_timer *t) {
    // Telemetry (500ms)
    printf("=== SENSOR DATA ===\n");
    printf("IR: L=%u R=%d (%.1fs)\n",
           ir_sensor_data.left_raw, ir_sensor_data.right_digital,
           (double)(time_us_64() - ir_sensor_data.last_change_time) / 1000000.0);
    printf("US: %.1fcm %s\n",
           ultrasonic_data.distance_cm, ultrasonic_data.object_present ? "OBSTACLE" : "CLEAR");
    printf("IMU: P=%.1f R=%.1f Y=%.1f\n",
           imu_data.pitch, imu_data.roll, imu_data.yaw);
    printf("ENC: E1=%ld E2=%ld\n", 
           (long)motor_data.encoder1_pulses, (long)motor_data.encoder2_pulses);
    printf("==================\n");
    return true; // Continue repeating
}

// ==============================
// TASK FUNCTIONS
// ==============================

void motor_control_task(__unused void *params) {
    printf("Motor control task started\n");
    
    char line[64];
    int idx = 0;
    
    while(true) {
        // Check for serial commands
        int ch = getchar_timeout_us(0);
        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                line[idx] = 0;
                idx = 0;
                if (line[0]) {
                    printf("Processing command: %s\n", line);
                    process_motor_command(line);
                }
            } else if (idx < (int)sizeof(line)-1) {
                line[idx++] = (char)ch;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void system_status_task(__unused void *params) {
    printf("System status task started\n");
    
    while(true) {
        // Simple heartbeat
        static int heartbeat = 0;
        heartbeat++;
        
        // Check if sensors are updating
        uint32_t current_time = time_us_32();
        if (current_time - motor_data.last_encoder_time > 1000000) {
            printf("[STATUS] Encoders not updating\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ==============================
// CONTROL FUNCTIONS
// ==============================

float calculate_pid_output(uint16_t ir_raw, float yaw) {
    float base_speed = 40.0f;
    float correction = (ir_raw - 2000) * 0.01f;
    
    if (correction > 25.0f) correction = 25.0f;
    if (correction < -25.0f) correction = -25.0f;
    
    return base_speed + correction;
}

void set_motor_speed(float speed) {
    float left_speed = speed;
    float right_speed = speed;
    
    float correction_factor = (ir_sensor_data.left_raw - 2000) * 0.02f;
    left_speed -= correction_factor;
    right_speed += correction_factor;
    
    // Limit speeds
    if (left_speed > 80.0f) left_speed = 80.0f;
    if (left_speed < -80.0f) left_speed = -80.0f;
    if (right_speed > 80.0f) right_speed = 80.0f;
    if (right_speed < -80.0f) right_speed = -80.0f;
    
    drive_signed(left_speed, right_speed);
}

void avoid_obstacle(void) {
    printf("Avoiding obstacle...\n");
    all_stop();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    drive_signed(-30.0f, -30.0f);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    drive_signed(40.0f, -40.0f);
    vTaskDelay(pdMS_TO_TICKS(800));
    
    drive_signed(40.0f, 40.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    all_stop();
    printf("Avoidance complete\n");
}

void decode_barcode(void) {
    static uint32_t last_barcode_time = 0;
    uint32_t current_time = time_us_32();
    
    if (current_time - last_barcode_time > 500000) {
        printf("Barcode decoding started\n");
        // Add barcode logic here
        printf("Barcode decoded\n");
        last_barcode_time = current_time;
    }
}

// ==============================
// INITIALIZATION FUNCTIONS
// ==============================

void initialize_pico_timers(void) {
    // Create Pico SDK repeating timers
    add_repeating_timer_ms(-20, read_imu_data_callback, NULL, &imu_timer);      // 20ms period
    add_repeating_timer_ms(-10, update_encoders_callback, NULL, &encoder_timer); // 10ms period
    add_repeating_timer_ms(-50, sensor_update_callback, NULL, &sensor_timer);   // 50ms period
    add_repeating_timer_ms(-500, telemetry_callback, NULL, &telemetry_timer);   // 500ms period
    
    printf("Pico SDK timers initialized and started\n");
}

void initialize_sensors(void) {
    // Initialize all sensors
    ultrasonic_init();
    ir_init(NULL);
    motor_encoder_init();
    
    // Initialize right IR digital pin
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);
    
    // Initialize encoder state tracking
    last_enc1_state = gpio_get(6);
    last_enc2_state = gpio_get(4);
    
    // Initialize sensor data
    ir_sensor_data.last_digital_state = gpio_get(RIGHT_IR_DIGITAL_PIN);
    ir_sensor_data.right_digital = ir_sensor_data.last_digital_state;
    ir_sensor_data.last_change_time = time_us_64();
    motor_data.last_encoder_time = time_us_32();
    
    printf("All sensors initialized\n");
}

// ==============================
// MAIN TASK
// ==============================

void main_task(__unused void *params) {
    printf("Main task started - initializing system...\n");
    
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi hardware\n");
        vTaskDelete(NULL);
        return;
    }
    
    printf("=== Pico SDK Timer-Based Robot Control ===\n");
    
    // Initialize everything
    initialize_sensors();
    initialize_pico_timers();
    
    printf("\n=== SYSTEM READY ===\n");
    printf("Pico SDK timers active:\n");
    printf("  Encoders: 10ms (100Hz)\n");
    printf("  IMU: 20ms (50Hz)\n");
    printf("  Sensors: 50ms (20Hz) - IR, Ultrasonic, PID\n");
    printf("  Telemetry: 500ms (2Hz)\n");
    printf("Type 'help' for motor commands\n\n");

    // Main task just waits - timers handle everything
    while(true) {
        printf("FUCKING WORK\n");
        // Flash LED to show system is alive
        static uint32_t last_led_toggle = 0;
        if (time_us_32() - last_led_toggle > 500000) { // 500ms
            static bool led_state = false;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            led_state = !led_state;
            last_led_toggle = time_us_32();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vLaunch(void) {
    printf("Creating FreeRTOS tasks...\n");
    
    // Create main system task (highest priority)
    if (xTaskCreate(main_task, "MainThread", configMINIMAL_STACK_SIZE * 2, NULL, 
                    tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
        printf("Failed to create main task!\n");
        return;
    }
    
    // Create motor control task
    if (xTaskCreate(motor_control_task, "MotorControl", configMINIMAL_STACK_SIZE, NULL, 
                    tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
        printf("Failed to create motor control task!\n");
        return;
    }
    
    // Create system status task (lowest priority)
    if (xTaskCreate(system_status_task, "SystemStatus", configMINIMAL_STACK_SIZE, NULL, 
                    tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("Failed to create system status task!\n");
        return;
    }

    // Create message buffer
    xControlMessageBuffer = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);

    printf("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();
    
    // Should never reach here
    printf("ERROR: FreeRTOS scheduler failed to start!\n");
}

// int main(void) {
//     stdio_init_all();
    
//     printf("\n\n=== Pico SDK Timer Robot Control System ===\n");
//     printf("Initializing...\n");
    
//     // Wait for serial to be ready - give more time
//     sleep_ms(2000);
    
//     printf("Starting FreeRTOS...\n");
//     vLaunch();
    
//     // If we get here, something went wrong
//     while(1) {
//         printf("System failed to start - halting\n");
//         sleep_ms(1000);
//     }
    
//     return 0;
// }