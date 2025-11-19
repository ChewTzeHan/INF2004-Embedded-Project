#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Include all necessary headers
#include "motor_encoder_demo.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include "barcode.h"
#include "Obstacle_Avoidance.h"
#include "PID_Line_Follow.h"
#include "mqtt_client.h"
#include "encoder.h"
#include "imu_raw_demo.h"

// Robot states
typedef enum {
    STATE_LINE_FOLLOWING,
    STATE_OBSTACLE_AVOIDANCE, 
    STATE_BARCODE_SCANNING,
    STATE_WAITING_FOR_JUNCTION,
    STATE_EXECUTING_TURN
} robot_state_t;

// Global variables for inter-task communication
static volatile robot_state_t current_state = STATE_LINE_FOLLOWING;
static volatile bool system_active = true;
static volatile bool obstacle_detected_flag = false;
static volatile char pending_turn_direction[10] = ""; // Store turn direction after barcode scan
static SemaphoreHandle_t obstacle_mutex;
static SemaphoreHandle_t state_mutex;
static SemaphoreHandle_t turn_mutex;

// Forward declarations
bool check_obstacle_detection(void);
void avoid_obstacle_only(void);
bool check_barcode_detection(void);
bool barcode_scan_only(barcode_result_t *result, char *nw_pattern, size_t pattern_size, 
                      char *timing_str, size_t timing_size, char *direction_str, size_t direction_size);

// Obstacle detection task
static void obstacle_detection_task(void *pv) {
    const TickType_t check_interval = pdMS_TO_TICKS(300); // Check every 300ms
    
    printf("[OBSTACLE_TASK] Obstacle detection task started\n");
    
    while (1) {
        // Use the fast detection function
        if (ultrasonic_detect_obstacle_fast()) {
            xSemaphoreTake(obstacle_mutex, portMAX_DELAY);
            obstacle_detected_flag = true;
            xSemaphoreGive(obstacle_mutex);
            printf("[OBSTACLE_TASK] Obstacle detected!\n");
        }
        vTaskDelay(check_interval);
    }
}

// Barcode detection task
static void barcode_detection_task(void *pv) {
    const TickType_t check_interval = pdMS_TO_TICKS(100); // Check every 100ms (barcode is fast)
    
    printf("[BARCODE_TASK] Barcode detection task started\n");
    
    while (1) {
        if (check_barcode_detection()) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if (current_state == STATE_LINE_FOLLOWING) {
                current_state = STATE_BARCODE_SCANNING;
                printf("[BARCODE_TASK] Barcode detected - switching to scanning mode\n");
                all_stop();
                sleep_ms(1000);
                
                drive_signed(-30*1.25f, -30); // Small reverse
                sleep_ms(500);
                all_stop();
                sleep_ms(1000);
            }
            xSemaphoreGive(state_mutex);
        }
        vTaskDelay(check_interval);
    }
}

// Junction detection task - waits for black line after barcode scan
static void junction_detection_task(void *pv) {
    const TickType_t check_interval = pdMS_TO_TICKS(50); // Check every 50ms for fast response
    
    printf("[JUNCTION_TASK] Junction detection task started\n");
    
    while (1) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool should_check_junction = (current_state == STATE_WAITING_FOR_JUNCTION);
        xSemaphoreGive(state_mutex);
        
        if (should_check_junction && check_barcode_detection()) {
            printf("[JUNCTION_TASK] Junction detected! Executing turn...\n");
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            current_state = STATE_EXECUTING_TURN;
            xSemaphoreGive(state_mutex);
        }
        vTaskDelay(check_interval);
    }
}

void execute_turn_command(const char* direction) {
    printf("Executing turn command: %s\n", direction);
    
    // Stop and stabilize before turn
    all_stop();
    sleep_ms(1000);
    
    if (strcmp(direction, "RIGHT") == 0) {
        printf("Turning RIGHT at junction\n");
        // Right turn sequence
        drive_signed(40, 40);
        sleep_ms(500);
        drive_signed(-40, 60); // Right turn
        sleep_ms(500);
    } else {
        printf("Turning LEFT at junction\n");
        // Left turn sequence (default)
        drive_signed(40, -40); // Left turn
        sleep_ms(500);
    }
    
    all_stop();
    sleep_ms(1500);
    printf("Turn completed\n");
}

// Main robot control task
// Main robot control task
static void robot_control_task(void *pv) {
    uint32_t last_telemetry_time = 0;
    uint32_t last_speed_update_time = 0;
    bool telemetry_initialized = false;
    bool obstacle_done = false;
    
    printf("[CONTROL_TASK] Starting main control loop\n");
    
    // Initialize speed calculation system (like obstacle_avoidance.c does)
    speed_calc_init();
    reset_total_distance();
    
    while (system_active) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Update speed and distance calculations (same method as obstacle_avoidance.c)
        if (now - last_speed_update_time >= 100) {
            update_speed_and_distance(); // This updates g_current_speed_cm_s and g_total_distance_cm
            last_speed_update_time = now;
            
            // Debug: Print speed/distance values to verify they're updating
            static uint32_t last_debug_time = 0;
            if (now - last_debug_time >= 1000) {
                printf("[SPEED_DEBUG] Speed: %.2f cm/s, Distance: %.2f cm\n", 
                       get_current_speed_cm_s(), get_total_distance_cm());
                last_debug_time = now;
            }
        }
        
        // Send telemetry periodically (every 2 seconds)
        if (mqtt_is_connected() && (now - last_telemetry_time >= 2000)) {
            // Get values using the same method as obstacle_avoidance.c
            float speed = get_current_speed_cm_s();
            float distance = get_total_distance_cm();
            float ultra = ultrasonic_get_distance_cm();
            
            // Get yaw from IMU using the same method as obstacle_avoidance.c
            float direction = 0.0f;
            float yaw = get_heading_fast(&direction);
            
            const char* state_str = "";
            switch (current_state) {
                case STATE_LINE_FOLLOWING: state_str = "LINE_FOLLOWING"; break;
                case STATE_OBSTACLE_AVOIDANCE: state_str = "OBSTACLE_AVOIDANCE"; break;
                case STATE_BARCODE_SCANNING: state_str = "BARCODE_SCANNING"; break;
                case STATE_WAITING_FOR_JUNCTION: state_str = "WAITING_FOR_JUNCTION"; break;
                case STATE_EXECUTING_TURN: state_str = "EXECUTING_TURN"; break;
            }
            
            // Debug output to verify values
            printf("[TELEMETRY] Speed: %.2f cm/s, Distance: %.2f cm, Yaw: %.2f°, Ultra: %.2f cm, State: %s\n", 
                   speed, distance, yaw, ultra, state_str);
            
            mqtt_publish_telemetry(speed, distance, yaw, ultra, state_str);
            last_telemetry_time = now;
            
            if (!telemetry_initialized) {
                printf("Telemetry system active\n");
                telemetry_initialized = true;
            }
        }
        
        // Get current state safely
        robot_state_t local_state;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        local_state = current_state;
        xSemaphoreGive(state_mutex);
        
        switch (local_state) {
            case STATE_LINE_FOLLOWING:
                // Normal line following - call this EVERY loop for smooth control
                follow_line_simple();
                
                // Check obstacle flag (very fast - just reading a variable)
                bool obstacle_found = false;
                xSemaphoreTake(obstacle_mutex, portMAX_DELAY);
                obstacle_found = obstacle_detected_flag;
                obstacle_detected_flag = false; // Reset flag
                xSemaphoreGive(obstacle_mutex);
                
                if (!obstacle_done && obstacle_found) {
                    printf("🚨 OBSTACLE DETECTED - Switching to avoidance mode\n");
                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                    current_state = STATE_OBSTACLE_AVOIDANCE;
                    xSemaphoreGive(state_mutex);
                    
                    all_stop();
                    sleep_ms(500);
                    
                    // Send telemetry with current values
                    float speed = get_current_speed_cm_s();
                    float distance = get_total_distance_cm();
                    float ultra = ultrasonic_get_distance_cm();
                    float direction = 0.0f;
                    float yaw = get_heading_fast(&direction);
                    
                    mqtt_publish_telemetry(speed, distance, yaw, ultra, "OBSTACLE_DETECTED");
                }
                break;
                
            case STATE_OBSTACLE_AVOIDANCE:
                {
                    // Execute obstacle avoidance
                    avoid_obstacle_only();
                    
                    // Return to line following
                    printf("✅ Obstacle avoidance completed - Returning to line following\n");
                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                    current_state = STATE_LINE_FOLLOWING;
                    xSemaphoreGive(state_mutex);
                    
                    // Send telemetry with current values
                    float speed = get_current_speed_cm_s();
                    float distance = get_total_distance_cm();
                    float ultra = ultrasonic_get_distance_cm();
                    float direction = 0.0f;
                    float yaw = get_heading_fast(&direction);
                    
                    mqtt_publish_telemetry(speed, distance, yaw, ultra, "AVOIDANCE_COMPLETE");
                    sleep_ms(1000);
                    obstacle_done = true;
                }
                break;
                
            case STATE_BARCODE_SCANNING:
                {
                    // Execute barcode scanning
                    float speed = get_current_speed_cm_s();
                    float distance = get_total_distance_cm();
                    float ultra = ultrasonic_get_distance_cm();
                    float direction = 0.0f;
                    float yaw = get_heading_fast(&direction);

                    mqtt_publish_telemetry(speed, distance, yaw, ultra, "SCANNING_BARCODE");
                    barcode_result_t scan_result;
                    char nw_pattern[100], timing_str[200], direction_str[10];
                    
                    if (barcode_scan_only(&scan_result, nw_pattern, sizeof(nw_pattern), 
                                        timing_str, sizeof(timing_str), direction_str, sizeof(direction_str))) {
                        printf("✅ Barcode decoded: '%s', Direction: %s\n", scan_result.data, direction_str);
                        
                        // Store the turn direction and wait for junction
                        xSemaphoreTake(turn_mutex, portMAX_DELAY);
                        strncpy(pending_turn_direction, direction_str, sizeof(pending_turn_direction) - 1);
                        xSemaphoreGive(turn_mutex);
                        
                        // Switch to waiting for junction
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_state = STATE_WAITING_FOR_JUNCTION;
                        xSemaphoreGive(state_mutex);
                        
                        printf("🔄 Waiting for junction to execute %s turn...\n", direction_str);
                        
                        // Send telemetry with current values
                        float speed = get_current_speed_cm_s();
                        float distance = get_total_distance_cm();
                        float ultra = ultrasonic_get_distance_cm();
                        float direction = 0.0f;
                        float yaw = get_heading_fast(&direction);
                        
                        char result_msg[100];
                        snprintf(result_msg, sizeof(result_msg), "BARCODE_SCANNED: %s -> %s (Waiting for junction)", 
                                scan_result.data, direction_str);
                        mqtt_publish_telemetry(speed, distance, yaw, ultra, result_msg);
                        
                    } else {
                        printf("❌ Barcode scan failed - Using default RIGHT turn\n");
                        
                        // Store default turn direction
                        xSemaphoreTake(turn_mutex, portMAX_DELAY);
                        strncpy(pending_turn_direction, "RIGHT", sizeof(pending_turn_direction) - 1);
                        xSemaphoreGive(turn_mutex);
                        
                        // Switch to waiting for junction
                        xSemaphoreTake(state_mutex, portMAX_DELAY);
                        current_state = STATE_WAITING_FOR_JUNCTION;
                        xSemaphoreGive(state_mutex);
                        
                        printf("🔄 Waiting for junction to execute default RIGHT turn...\n");
                        
                        // Send telemetry with current values
                        float speed = get_current_speed_cm_s();
                        float distance = get_total_distance_cm();
                        float ultra = ultrasonic_get_distance_cm();
                        float direction = 0.0f;
                        float yaw = get_heading_fast(&direction);
                        
                        char result_msg[100];
                        snprintf(result_msg, sizeof(result_msg), "BARCODE_SCANNED: C -> RIGHT (Waiting for junction)");
                        mqtt_publish_telemetry(speed, distance, yaw, ultra, result_msg);
                    }
                }
                break;
                
            case STATE_WAITING_FOR_JUNCTION:
                // Continue line following while waiting for junction
                follow_line_simple();
                break;
                
            case STATE_EXECUTING_TURN:
                {
                    // Get the stored turn direction
                    char turn_dir[10];
                    xSemaphoreTake(turn_mutex, portMAX_DELAY);
                    strncpy(turn_dir, pending_turn_direction, sizeof(turn_dir));
                    xSemaphoreGive(turn_mutex);
                    
                    // Execute the turn
                    execute_turn_command(turn_dir);
                    
                    // Return to line following
                    printf("Returning to line following mode\n");
                    xSemaphoreTake(state_mutex, portMAX_DELAY);
                    current_state = STATE_LINE_FOLLOWING;
                    xSemaphoreGive(state_mutex);
                    
                    // Send telemetry with current values
                    float speed = get_current_speed_cm_s();
                    float distance = get_total_distance_cm();
                    float ultra = ultrasonic_get_distance_cm();
                    float direction = 0.0f;
                    float yaw = get_heading_fast(&direction);
                    
                    char turn_msg[100];
                    snprintf(turn_msg, sizeof(turn_msg), "TURN_COMPLETED: %s", turn_dir);
                    mqtt_publish_telemetry(speed, distance, yaw, ultra, turn_msg);
                    
                    sleep_ms(1000);
                }
                break;
        }
        
        // Small delay to yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    vTaskDelete(NULL);
}
void initialize_all_systems(void) {
    stdio_init_all();
    sleep_ms(2000);
    
    printf("=== INTEGRATED ROBOT SYSTEM INITIALIZATION ===\n");
    
    // Create mutexes FIRST
    obstacle_mutex = xSemaphoreCreateMutex();
    state_mutex = xSemaphoreCreateMutex();
    turn_mutex = xSemaphoreCreateMutex();
    
    if (obstacle_mutex == NULL || state_mutex == NULL || turn_mutex == NULL) {
        printf("ERROR: Failed to create mutexes\n");
        return;
    }
    
    // Initialize all subsystems
    motor_encoder_init();
    ultrasonic_init();
    ir_init(NULL);
    barcode_init();
    
    // Initialize speed calculation system (CRITICAL - this initializes encoders properly)
    speed_calc_init();
    
    // Initialize obstacle avoidance systems
    
    imu_init();
    servo_init();
    printf("All hardware systems initialized successfully!\n");
}

// WiFi connection task
static void wifi_connection_task(void *pv) {
    printf("[WIFI_TASK] Starting WiFi connection...\n");
    
    // Try to connect to WiFi (blocking call)
    bool wifi_connected = wifi_and_mqtt_start();
    
    if (wifi_connected) {
        printf("[WIFI_TASK] ✅ WiFi connected successfully!\n");
        //printf("[WIFI_TASK] Local IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    } else {
        printf("[WIFI_TASK] ❌ WiFi connection failed, continuing without network\n");
    }
    
    // Now that WiFi is done (success or failure), start the robot tasks
    printf("[WIFI_TASK] Starting robot tasks...\n");
    
    // Create detection tasks with higher priority than control task
    xTaskCreate(obstacle_detection_task, "obstacle_det", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(barcode_detection_task, "barcode_det", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(junction_detection_task, "junction_det", 1024, NULL, tskIDLE_PRIORITY + 2, NULL);
    
    // Create control task with normal priority
    xTaskCreate(robot_control_task, "robot_control", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);
    
    printf("[WIFI_TASK] All robot tasks created. Starting in LINE FOLLOWING mode...\n");
    
    // Delete this WiFi task since it's no longer needed
    vTaskDelete(NULL);
}

static void init_task(void *pv) {
    vTaskDelay(pdMS_TO_TICKS(100));
    initialize_all_systems();
    
    // Create WiFi connection task (this will create robot tasks when done)
    xTaskCreate(wifi_connection_task, "wifi_conn", 2048, NULL, tskIDLE_PRIORITY + 3, NULL);
    
    printf("[INIT_TASK] WiFi connection task created. Waiting for network...\n");
    
    // Delete this initialization task since it's no longer needed
    vTaskDelete(NULL);
}

int main(void) {
    // Create a single initialization task
    xTaskCreate(init_task, "init", 2048, NULL, tskIDLE_PRIORITY + 3, NULL);
    
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (1) { 
        tight_loop_contents();
    }
    
    return 0;
}