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
#include "imu_raw_demo.h"  // IMU sensor header
#include "ir_sensor.h"     // ADD THIS - IR sensor header
#include "encoder.h"       // ADD THIS - Encoder header
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Pico-specific includes for WiFi and hardware
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

// FreeRTOS includes for real-time operating system
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

// Header file for server functions
#include "picow_freertos_ping.h"
#include "motor_encoder_demo.h"
#include "ultrasonic.h"  // Ultrasonic sensor header

// Hardware peripheral includes
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

// ==============================
// GLOBAL VARIABLES
// ==============================

ir_calib_t ir_calibration;  // ADD THIS - IR sensor calibration data

// ==============================
// TASK DEFINITIONS
// ==============================

/**
 * @brief Motor control task
 * 
 * Handles motor commands from serial input and displays encoder readings
 * 
 * @param params Task parameters (not used)
 */
void motor_task(__unused void *params) {
    motor_encoder_init();
    print_motor_help();
    
    char line[64]; 
    int idx = 0;
    
    while (true) {
        int ch = getchar_timeout_us(0);
        
        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                line[idx] = 0; 
                idx = 0;
                if (line[0]) {
                    process_motor_command(line);
                }
            } else if (idx < (int)sizeof(line)-1) {
                line[idx++] = (char)ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief Motor encoder polling task
 * 
 * Periodically polls encoder states (replaces interrupts)
 * 
 * @param params Task parameters (not used)
 */
void motor_encoder_poll_task(__unused void *params) {
    while(true) {
        motor_encoder_poll();  // Poll both encoders
        vTaskDelay(pdMS_TO_TICKS(1));  // Poll at ~1kHz (adjust as needed)
    }
}

/**
 * @brief Sensor data aggregation and display task
 * 
 * Collects data from all sensors and prints in a unified format
 * 
 * @param params Task parameters (not used)
 */
void sensor_display_task(__unused void *params) {
    // Initialize sensors
    ultrasonic_init();
    ir_init(&ir_calibration);
    
    // Quick IR calibration
    for (int i = 0; i < 20; i++) {
        uint16_t v = ir_read_raw();
        ir_update_calibration(&ir_calibration, v);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    printf("=== Sensor System Initialized ===\n");
    printf("Ultrasonic: TRIG GP%d, ECHO GP%d\n", TRIG_PIN, ECHO_PIN);
    printf("IR Sensor: GPIO%d (ADC%d)\n", IR_GPIO, IR_ADC_INPUT);
    printf("Motor Encoders: ENC1 GP6, ENC2 GP4\n");
    printf("IMU: I2C0 (SDA GP16, SCL GP17)\n\n");
    
    // Print header
    printf("TIME | ULTRASONIC | IR_SENSOR | MOTOR_ENC | IMU_DATA\n");
    printf("-----|------------|-----------|-----------|-----------------------------\n");
    
    uint32_t iteration = 0;
    static int ir_prev_state = 0;
    
    while(true) {
        iteration++;
        
        // Collect sensor data
        float distance = ultrasonic_get_distance_cm();
        uint16_t ir_raw = ir_read_raw();
        printf("IR RAW %4u |\n", ir_raw); // DEBUG: Print raw IR value
        ir_update_calibration(&ir_calibration, ir_raw);
        int ir_state = ir_classify_hysteresis(ir_raw, &ir_calibration, ir_prev_state);
        ir_prev_state = ir_state;
        const char* surface_type = (ir_state == 1) ? "BLACK" : "WHITE";
        
        // Print unified sensor data
        printf("%4lu |", (unsigned long)iteration);
        
        // Ultrasonic data
        printf("RAW DIST %5.1f |\n", distance); // DEBUG: Print raw distance value
        if (distance > 0 && distance <= 100.0f) {
            printf(" %5.1f cm   |", distance);
        } else {
            printf(" NO_OBST   |");  // FIXED: Added parentheses
        }
        
        // IR sensor data
        printf(" %3s(%4u)|", surface_type, ir_raw);  // FIXED: Added parentheses
        
        // Motor encoder data
        printf(" %2lu/%2lu   |",  // FIXED: Added parentheses
               (unsigned long)E1.edges, (unsigned long)E2.edges);
        
        // IMU data (if available)
        compute_and_print_imu_data();  // This will print on the same line
        
        printf("\n");  // Add newline after complete sensor reading
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Update every 500ms
    }
}

/**
 * @brief LED blinking task
 * 
 * Blinks the onboard LED to indicate the system is running.
 * 
 * @param params Task parameters (not used)
 */
void led_task(__unused void *params) {
    while(true) {
        vTaskDelay(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(1000);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    }
}

/**
 * @brief Modified main task function with WiFi disabled
 * 
 * This task initializes the system but skips WiFi connection for testing
 * 
 * @param params Task parameters (not used)
 */
void main_task(__unused void *params) {
    // Initialize the Pico W's WiFi hardware
    if (cyw43_arch_init()) {
        printf("Failed to initialize WiFi hardware\n");
        return;
    }
    
    printf("WiFi hardware initialized (connection disabled for testing)\n");
    printf("All sensor systems are active:\n");
    printf("- Motor control with encoders\n");
    printf("- Ultrasonic distance sensor\n");
    printf("- IMU (Accelerometer + Magnetometer)\n");
    printf("- IR line following sensor\n");
    printf("- Digital encoder input\n");

    // Main system loop - keep the system running
    while(true) {
        vTaskDelay(1000);
    }

    // Clean up if we ever exit (unlikely)
    cyw43_arch_deinit();
}

/**
 * @brief FreeRTOS launch function
 * 
 * Creates all tasks and starts the FreeRTOS scheduler.
 */
void vLaunch(void) {
    TaskHandle_t task;
    
    // Create main system task (WiFi disabled)
    xTaskCreate(main_task, "MainThread", configMINIMAL_STACK_SIZE, NULL, 
                TEST_TASK_PRIORITY, &task);
    
    // Create LED blinking task
    TaskHandle_t ledtask;
    xTaskCreate(led_task, "LedThread", configMINIMAL_STACK_SIZE, NULL, 8, &ledtask);
    
    // Create sensor display task (replaces individual sensor tasks)
    TaskHandle_t sensordisplaytask;
    xTaskCreate(sensor_display_task, "SensorDisplayThread", configMINIMAL_STACK_SIZE * 2, NULL, 7, &sensordisplaytask);

    // Create motor control task
    TaskHandle_t motortask;
    xTaskCreate(motor_task, "MotorThread", configMINIMAL_STACK_SIZE, NULL, 6, &motortask);

    // Create motor encoder POLLING task (replaces interrupt approach)
    TaskHandle_t encoderpolltask;
    xTaskCreate(motor_encoder_poll_task, "EncoderPollThread", configMINIMAL_STACK_SIZE, NULL, 5, &encoderpolltask);

    // Create IMU sensor task
    TaskHandle_t imutask;
    xTaskCreate(imu_task, "ImuThread", configMINIMAL_STACK_SIZE * 2, NULL, 3, &imutask);

    // Create message buffer for inter-task communication
    xControlMessageBuffer = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);

// Core affinity configuration for multi-core systems
#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    vTaskCoreAffinitySet(task, 1);
#endif

    // Start the FreeRTOS scheduler - this never returns
    vTaskStartScheduler();
}

/**
 * @brief Main application entry point
 * 
 * Initializes hardware and starts the FreeRTOS system.
 * 
 * @return int Exit code (should never return due to FreeRTOS scheduler)
 */
int main(void) {
    // Initialize standard I/O (USB serial)
    stdio_init_all();
    
    printf("=== Pico Multi-Sensor Robot Control System ===\n");
    printf("Testing all integrated systems:\n");
    printf("- Motor control with encoders\n");
    printf("- Ultrasonic obstacle detection\n");
    printf("- IMU orientation and motion\n");
    printf("- IR line following\n");
    printf("- Digital encoder input\n");
    printf("WiFi connection disabled for testing\n");
    printf("Waiting for serial port...\n");
    
    // Wait for serial connection to be established
    sleep_ms(2000);

    // Determine FreeRTOS variant name for display
    const char *rtos_name;
#if (portSUPPORT_SMP == 1)
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

    printf("Starting %s on core 0:\n", rtos_name);
    printf("Motor commands available via serial monitor\n");
    printf("Type 'help' for motor command list\n");
    printf("All sensor readings will be displayed in unified format\n\n");
    
    // Launch the FreeRTOS system (this function never returns)
    vLaunch();
    
    return 0;
}