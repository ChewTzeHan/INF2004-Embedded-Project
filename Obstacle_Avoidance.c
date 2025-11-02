#include "Obstacle_Avoidance.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "ultrasonic.h"
#include "motor_encoder_demo.h"
#include "ir_sensor.h"
#include "PID_Line_Follow.h"
#include "ir_sensor.h"
#include "mqtt_client.h"
#include "encoder.h"  // Add this include for the new encoder functions

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "imu_raw_demo.h"

// ==============================
// PID LINE FOLLOWING CONSTANTS
// ==============================

// #define TURN_90_DEG_PULSES  8  // Adjust based on your encoder counts
// #define TURN_180_DEG_PULSES  16 // Adjust based on your encoder counts
// #define TURN_45_DEG_PULSES   4  // Adjust based on your encoder counts

// PID global variables


// ==============================
// ENCODER-BASED TURNING CONFIG
// ==============================

// REMOVED OLD ENCODER VARIABLES - using new interrupt-based ones instead
// volatile uint32_t encoder_left_count = 0;
// volatile uint32_t encoder_right_count = 0;
// volatile bool last_enc1_state = false;
// volatile bool last_enc2_state = false;

static volatile uint32_t g_last_pub_ms = 0;

// ==============================
// ENCODER-BASED SPEED & DISTANCE CALCULATION
// ==============================

// Configuration constants - use the ones from encoder.h instead
// #define WHEEL_CIRCUMFERENCE_CM 6.5f  // Now defined in encoder.h
// #define PULSES_PER_REVOLUTION 20     // Now defined in encoder.h
// #define CM_PER_PULSE (WHEEL_CIRCUMFERENCE_CM / PULSES_PER_REVOLUTION)

// Global variables for speed and distance calculation - using new encoder system
static volatile uint32_t g_last_speed_calc_time = 0;
static volatile float g_current_speed_cm_s = 0.0f;
static volatile float g_total_distance_cm = 0.0f;
static volatile float g_last_distance_cm = 0.0f;

// Initialize speed calculation using new encoder system
void speed_calc_init(void) {
    // Use the new interrupt-based encoder system
    encoders_init(true);  // Initialize encoders with pull-up
    
    g_last_speed_calc_time = to_ms_since_boot(get_absolute_time());
    g_current_speed_cm_s = 0.0f;
    g_total_distance_cm = 0.0f;
    g_last_distance_cm = 0.0f;
    printf("[SPEED_CALC] Initialized with interrupt-based encoders and manual speed calculation\n");
}

// Calculate speed and update total distance using distance-over-time method
void update_speed_and_distance(void) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    uint32_t time_elapsed_ms = current_time - g_last_speed_calc_time;
    
    if (time_elapsed_ms < 100) return; // Wait for at least 100ms for meaningful speed calculation
    
    // Get current distance using new interrupt-based functions
    float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float current_distance = (left_distance + right_distance) / 2.0f;
    
    // Calculate distance traveled since last measurement
    float distance_traveled = current_distance - g_last_distance_cm;
    
    // Calculate speed (cm/s) = distance (cm) / time (s)
    float time_elapsed_s = time_elapsed_ms / 1000.0f;
    
    if (time_elapsed_s > 0.0f) {
        g_current_speed_cm_s = distance_traveled / time_elapsed_s;
        
        // Add sanity checks for reasonable speed values
        if (g_current_speed_cm_s < 0) {
            g_current_speed_cm_s = 0.0f; // No negative speeds
        }
        if (g_current_speed_cm_s > 200.0f) { // Max reasonable speed for small robot
            g_current_speed_cm_s = 200.0f;
        }
    } else {
        g_current_speed_cm_s = 0.0f;
    }
    
    // Update total distance
    g_total_distance_cm = current_distance;
    
    // Debug output
    printf("SPEED CALC: dist_traveled=%.2fcm, time=%.3fs, speed=%.2fcm/s\n", 
           distance_traveled, time_elapsed_s, g_current_speed_cm_s);
    
    // Update for next calculation
    g_last_distance_cm = current_distance;
    g_last_speed_calc_time = current_time;
}

// Get current speed in cm/s
float get_current_speed_cm_s(void) {
    return g_current_speed_cm_s;

}

// Get total distance traveled in cm
float get_total_distance_cm(void) {
    // Use the new interrupt-based distance calculation
    float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    return (left_distance + right_distance) / 2.0f;
}

// Reset total distance counter
void reset_total_distance(void) {
    // Use the new interrupt-based reset function
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    g_total_distance_cm = 0.0f;
    printf("[SPEED_CALC] Total distance reset using interrupt-based encoders\n");
}

// ==============================
// SERVO CONTROL FUNCTIONS
// ==============================

void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 20000);
    
    pwm_init(slice_num, &config, true);
    servo_set_angle(85.0f);
    printf("[SERVO] Initialized on GPIO %d\n", SERVO_PIN);
}

void servo_set_angle(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    
    float pulse_width_us = SERVO_MIN_PULSE_US + (angle / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    uint16_t level = (uint16_t)pulse_width_us;
    pwm_set_gpio_level(SERVO_PIN, level);
}

// ==============================
// REAL-TIME ENCODER POLLING FUNCTIONS - REPLACED WITH INTERRUPT-BASED
// ==============================

// REMOVED OLD POLLING FUNCTIONS - using interrupt-based instead
/*
void update_encoder_counts(void) {
    // This function is no longer needed - interrupts handle everything
}

void reset_encoder_counts(void) {
    // Use the new interrupt-based reset function instead
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
}

uint32_t get_encoder_pulses(void) {
    // Use the new interrupt-based pulse count function
    int32_t left_pulses = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
    int32_t right_pulses = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);
    return (left_pulses + right_pulses) / 2;
}
*/

void print_encoder_status(void) {
    // Use the new interrupt-based encoder functions
    int32_t left_pulses = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
    int32_t right_pulses = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);
    float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float left_speed = encoder_get_speed_cm_s_timeout(ENCODER_LEFT_GPIO, 200);
    float right_speed = encoder_get_speed_cm_s_timeout(ENCODER_RIGHT_GPIO, 200);
    
    printf("[ENCODER] Left: %ld pulses, %.1f cm, %.1f cm/s | Right: %ld pulses, %.1f cm, %.1f cm/s\n", 
           left_pulses, left_distance, left_speed,
           right_pulses, right_distance, right_speed);
}

// ==============================
// OBSTACLE DETECTION
// ==============================

bool obstacle_detected(float distance_cm) {
    if (distance_cm > 2.0f && distance_cm <= OBSTACLE_DETECTION_DISTANCE_CM) {
        return true;
    }
    return false;
}

bool object_too_close(float distance_cm) {
    if (distance_cm > 2.0f && distance_cm <= SAFE_DISTANCE_CM) {
        return true;
    }
    return false;
}

bool object_at_stop_distance(float distance_cm) {
    if (distance_cm > 2.0f && distance_cm <= INITIAL_STOP_DISTANCE_CM) {
        return true;
    }
    return false;
}

// ==============================
// OBJECT WIDTH SCANNING FUNCTIONS
// ==============================

float calculate_object_width(float distance_left, float distance_right, float servo_angle_left, float servo_angle_right) {
    float angle_C = fabsf((servo_angle_left+20.0f) - (servo_angle_right-20.0f)) * (float)M_PI / 180.0f;
    
    float a_squared = distance_left * distance_left;
    float b_squared = distance_right * distance_right;
    float two_ab_cosC = 2.0f * distance_left * distance_right * cosf(angle_C);
    
    float c_squared = a_squared + b_squared - two_ab_cosC;
    
    if (c_squared < 0) {
        c_squared = fabsf(c_squared);
    }
    
    float width = sqrtf(c_squared);
    
    printf("[WIDTH CALC] Left: %.1fcm @ %.1f°, Right: %.1fcm @ %.1f°, Width: %.1fcm\n",
           distance_left, servo_angle_left, distance_right, servo_angle_right, width);
    
    return width;
}

ObjectScanResult scan_object_width(void) {
    printf("[SCAN] Starting object width scan\n");
    
    ObjectScanResult result;
    result.left_distance = -1.0f;
    result.right_distance = -1.0f;
    result.left_angle = -1.0f;
    result.right_angle = -1.0f;
    result.width_cm = -1.0f;
    
    const float SCAN_STEP = 1.0f;
    const float OBJECT_DETECTION_THRESHOLD = 30.0f;
    
    float max_left_distance = -1.0f;
    float max_right_distance = -1.0f;
    float max_left_angle = -1.0f;
    float max_right_angle = -1.0f;
    bool left_found = false;
    servo_set_angle(150.0f);
    sleep_ms(500);
    for (float angle = 150.0f; angle >= 30.0f; angle -= SCAN_STEP) {
        servo_set_angle(angle);
        sleep_ms(75);
        
        float distance = ultrasonic_get_distance_cm();
        printf("[SCAN] Angle: %.1f°, Distance: %.1f cm\n", angle, distance);
        
        if (distance > 2.0f && distance <= 50.0f) {
            if (angle > 90.0f && !left_found) {
                if (distance > max_left_distance) {
                    max_left_distance = distance;
                    max_left_angle = angle;
                    left_found = true;
                }
            } else if (angle <= 90.0f) {
                if (distance > max_right_distance) {
                    max_right_distance = distance;
                    max_right_angle = angle;
                }
            }
        }
    }
    
    if (max_left_distance > 0 && max_right_distance > 0) {
        result.left_distance = max_left_distance;
        result.left_angle = max_left_angle;
        result.right_distance = max_right_distance;
        result.right_angle = max_right_angle;
        result.width_cm = calculate_object_width(result.left_distance, result.right_distance, 
                                               result.left_angle, result.right_angle);
    }
    
    servo_set_angle(85.0f);
    return result;
}

// ==============================
// ENCODER-BASED TURNING FUNCTIONS - UPDATED TO USE NEW ENCODER SYSTEM
// ==============================
#define sleep_duration_ms 500

// Helper function to get average pulses using new encoder system
uint32_t get_average_pulses(void) {
    int32_t left_pulses = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
    int32_t right_pulses = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);
    return (left_pulses + right_pulses) / 2;
}

void turn_left_encoder_based(uint32_t target_pulses) {
    printf("[TURN] Turning LEFT for %lu encoder pulses\n", target_pulses);
    
    // Reset encoder counts using new system
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    
    float turn_speed = 50.0f;
    drive_signed(-turn_speed, turn_speed);
    sleep_ms(sleep_duration_ms);
    
    all_stop();
}

void turn_right_encoder_based(uint32_t target_pulses) {
    printf("[TURN] Turning RIGHT for %lu encoder pulses\n", target_pulses);
    
    // Reset encoder counts using new system
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    
    float turn_speed = 50.0f;
    drive_signed(turn_speed, -turn_speed);
    sleep_ms(sleep_duration_ms);
    
    all_stop();
}

void turn_left_90_degrees(void) {
    turn_left_encoder_based(TURN_90_DEG_PULSES);
}

void turn_right_90_degrees(void) {
    turn_right_encoder_based(TURN_90_DEG_PULSES);
}

void turn_left_45_degrees(void) {
    turn_left_encoder_based(TURN_45_DEG_PULSES);
}

// ==============================
// PID LINE FOLLOWING FUNCTIONS
// ==============================


// ==============================
// CHECK FOR OBJECT FUNCTION
// ==============================

bool check_if_object_still_there(void) {
    printf("[CHECK] Checking if object is still present...\n");
    
    printf("[CHECK] Turning right to face object direction\n");
    turn_right_90_degrees();
    sleep_ms(500);
    
    float distance = ultrasonic_get_distance_cm();
    printf("[CHECK] Distance after turn: %.1f cm\n", distance);
    
    bool object_present = obstacle_detected(distance);
    
    if (object_present) {
        printf("[CHECK] Object is still present at %.1f cm\n", distance);
        printf("[CHECK] Turning left to continue circling\n");
        turn_left_90_degrees();
    } else {
        printf("[CHECK] Object is no longer present\n");
        printf("[CHECK] Turning left to continue pattern\n");
    }
    
    return object_present;
}

// ==============================
// ENCIRCLING FUNCTION WITH CHECKING - UPDATED TO USE NEW ENCODER SYSTEM
// ==============================

void encircle_object_with_checking(void) {
    printf("[ENCIRCLE] Starting object circling with periodic checks\n");
    
    bool continue_circling = true;
    uint32_t check_counter = 0;
    uint32_t no_object_counter = 0;
    const uint32_t REQUIRED_NO_OBJECT_CHECKS = 2;
    
    while (continue_circling) {
        check_counter++;
        printf("\n[ENCIRCLE] Check cycle %d (No-object count: %d/%d)\n", 
               check_counter, no_object_counter, REQUIRED_NO_OBJECT_CHECKS);
        
        printf("[ENCIRCLE] Moving forward for 2.5 seconds\n");
        drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
        sleep_ms(1150);

        uint32_t last_print_time = 0;
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        
        #define target_pulses 12
        while (get_average_pulses() < target_pulses) {
            // No need to call update_encoder_counts() - interrupts handle it automatically
            
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - last_print_time >= 100) {
                printf("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                    get_average_pulses(), target_pulses, current_time - start_time);
                last_print_time = current_time;
                
                if ((current_time - start_time) > 3000 && get_average_pulses() == 0) {
                    printf("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                    printf("[TURN] Falling back to timed turn\n");
                    break;
                }
            }
            
            sleep_ms(10);
        }

        all_stop();
        sleep_ms(3000);
        
        printf("[ENCIRCLE] Checking if object is still present\n");
        bool object_still_there = check_if_object_still_there();
        
        if (object_still_there) {
            printf("[ENCIRCLE] Object still detected, continuing to circle\n");
            printf("[ENCIRCLE] No-object counter reset to 0\n");
        } else {
            no_object_counter++;
            printf("[ENCIRCLE] Object not detected - no-object counter: %d/%d\n", 
                   no_object_counter, REQUIRED_NO_OBJECT_CHECKS);
            
            if (no_object_counter >= REQUIRED_NO_OBJECT_CHECKS) {
                printf("[ENCIRCLE] ✅ Object cleared for %d consecutive checks!\n", REQUIRED_NO_OBJECT_CHECKS);
                printf("[ENCIRCLE] Moving straight forward to continue path\n");
                uint32_t line_search_start_time = to_ms_since_boot(get_absolute_time());
                const uint32_t MAX_SEARCH_TIME_MS = 10000; // 10 second timeout
                bool line_detected = false;

                while (!line_detected && (to_ms_since_boot(get_absolute_time()) - line_search_start_time < MAX_SEARCH_TIME_MS)) {
                // Read IR sensor
                uint16_t ir_raw = ir_read_raw();
                int color_state = classify_colour(ir_raw);
                
                // Debug output (optional - can be removed)
                static uint32_t last_debug_time = 0;
                uint32_t current_time = to_ms_since_boot(get_absolute_time());
                if (current_time - last_debug_time > 500) {
                    printf("[LINE_SEARCH] IR Raw: %u, Color: %d\n", ir_raw, color_state);
                    last_debug_time = current_time;
                }
                
                // Check for black line (color_state 1 = Black according to your function)
                if (color_state == 1) {
                    line_detected = true;
                    printf("[ENCIRCLE] 🎯 BLACK LINE DETECTED! Raw: %u, Stopping.\n", ir_raw);
                    break;
                }
                
                // Continue driving forward
                drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
                sleep_ms(50);
            } 
            all_stop();
                
                continue_circling = false;
                printf("[ENCIRCLE] Object avoidance completed successfully!\n");
            } else {
                printf("[ENCIRCLE] Object not detected, but need %d more check(s) to confirm\n", 
                       REQUIRED_NO_OBJECT_CHECKS - no_object_counter);
                printf("[ENCIRCLE] Continuing circling pattern...\n");
            }
        }
        
        if (check_counter >= 15) {
            printf("[ENCIRCLE] ⚠️  Safety limit reached (15 checks)\n");
            printf("[ENCIRCLE] Moving straight forward to break possible loop\n");
            drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
            sleep_ms(3000);
            all_stop();
            continue_circling = false;
        }
        
        sleep_ms(500);
    }
    
    all_stop();
    printf("[ENCIRCLE] Object circling completed after %d checks\n", check_counter);
}

// ==============================
// COMPLETE AVOIDANCE CYCLE
// ==============================
float yaw = 0.0f;
float speed = 0.0f;
float dist = 0.0f;
float ultra = 0.0f;
void complete_avoidance_cycle(void) {
    printf("[CYCLE] Starting complete avoidance cycle\n");
    
    bool cycle_active = true;
    
    while (cycle_active) {
        printf("\n=== NEW CYCLE STARTING ===\n");
        
        printf("[CYCLE] Phase 1: Approaching object\n");
        bool approach_active = true;
        bool object_initially_detected = false;
        
        while (approach_active) {
            float distance = ultrasonic_get_distance_cm();
            
            if (distance < 0) {
                if (!object_initially_detected) {
                    drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
                }
            } 
            else if (object_at_stop_distance(distance)) {
                printf("[CYCLE] Object at stop distance (%.1f cm)! Stopping\n", distance);
                all_stop();
                approach_active = false;
                object_initially_detected = true;
                
                printf("[CYCLE] Scanning object width\n");
                mqtt_publish_telemetry(speed, dist, yaw, ultra, "scanning object");
                ObjectScanResult scan_result = scan_object_width();
                
                if (scan_result.width_cm > 0) {
                    printf("[CYCLE] Object width: %.1f cm\n", scan_result.width_cm);
                    char width_str[30];
                    snprintf(width_str, sizeof(width_str), "object width: %.1fcm", scan_result.width_cm);
                    mqtt_publish_telemetry(speed, dist, yaw, ultra, width_str);
            
                }
                
            }
            else if (obstacle_detected(distance)) {
                drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
                object_initially_detected = true;
            }
            else {
                drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
            }
            
            sleep_ms(50);
        }
        
        // printf("[CYCLE] Phase 2: Turning left to start circling\n");
        // sleep_ms(1000);
        // turn_left_90_degrees();
        // sleep_ms(1000);
        
        // printf("[CYCLE] Phase 3: Circling object with periodic checks\n");
        // encircle_object_with_checking();
        
        // printf("[CYCLE] Object avoidance completed!\n");
        // printf("[CYCLE] Robot has successfully navigated around the object.\n");
        // printf("[CYCLE] Stopping for now. Restart program to begin new cycle.\n");
        
        cycle_active = false;
        servo_set_angle(85.0f);
        sleep_ms(5000);
    }
}

// ==============================
// COMBINED LINE FOLLOWING WITH OBSTACLE AVOIDANCE
// ==============================

// ==============================
// ENCODER DEBUGGING FUNCTIONS - UPDATED TO USE NEW ENCODER SYSTEM
// ==============================

void debug_encoder_pulses(const char* context) {
    static uint32_t last_debug_time = 0;
    static int32_t last_left_count = 0;
    static int32_t last_right_count = 0;
    
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Use the new interrupt-based encoder functions - no need to call update_encoder_counts()
    int32_t left_count = encoder_get_pulse_count(ENCODER_LEFT_GPIO);
    int32_t right_count = encoder_get_pulse_count(ENCODER_RIGHT_GPIO);
    float left_distance = encoder_get_distance_cm(ENCODER_LEFT_GPIO);
    float right_distance = encoder_get_distance_cm(ENCODER_RIGHT_GPIO);
    float left_speed = encoder_get_speed_cm_s_timeout(ENCODER_LEFT_GPIO, 200);
    float right_speed = encoder_get_speed_cm_s_timeout(ENCODER_RIGHT_GPIO, 200);
    
    // Calculate pulse differences since last debug
    int32_t left_diff = left_count - last_left_count;
    int32_t right_diff = right_count - last_right_count;
    
    printf("[ENCODER_DEBUG][%s] Time: %lums\n", context, current_time);
    printf("  Left: %ld (+%ld) pulses, %.1f cm, %.1f cm/s\n", 
           left_count, left_diff, left_distance, left_speed);
    printf("  Right: %ld (+%ld) pulses, %.1f cm, %.1f cm/s\n", 
           right_count, right_diff, right_distance, right_speed);
    printf("  Average: %ld pulses, %.1f cm, %.1f cm/s\n",
           (left_count + right_count) / 2, 
           (left_distance + right_distance) / 2.0f,
           (left_speed + right_speed) / 2.0f);
    
    // Update for next calculation
    last_left_count = left_count;
    last_right_count = right_count;
    last_debug_time = current_time;
}

// High-frequency encoder monitoring
void monitor_encoder_continuously(uint32_t interval_ms) {
    static uint32_t last_monitor_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    if (current_time - last_monitor_time >= interval_ms) {
        debug_encoder_pulses("CONTINUOUS");
        last_monitor_time = current_time;
    }
}   

#define MAX_CONNECTION_ATTEMPTS 3

void line_follow_with_obstacle_avoidance(void) {
    printf("\n=== COMBINED LINE FOLLOWING WITH OBSTACLE AVOIDANCE ===\n");
    printf("Behavior: Line follow until object detected -> Avoid object -> Resume line following\n");
    
    bool system_active = true;
    bool avoiding_obstacle = false;

    // Start Wi-Fi + MQTT (safe: it locks around lwIP calls inside)
    bool connected = false;
    int attempts = 0;

    if (!wifi_and_mqtt_start()) {
        printf("[NET] Wi-Fi/MQTT start failed (continuing without MQTT)\n");
    }

    if (!connected) {
        printf("[NET] Wi-Fi/MQTT start failed after %d attempts (continuing without MQTT)\n", 
               MAX_CONNECTION_ATTEMPTS);
    } else {
        printf("[NET] Wi-Fi/MQTT connected successfully\n");
    }
    
    printf("[NET] Local IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    
    sleep_ms(2000);
    // Initialize systems
    ultrasonic_init();
    motor_encoder_init();
    servo_init();
    ir_init(NULL);
    
    // Initialize the new interrupt-based encoder system
    speed_calc_init();  // This calls encoders_init(true) internally
    
    imu_init();
    
    sleep_ms(2000);
    printf("Starting combined operation...\n");

    uint32_t last_encoder_debug_time = 0;
    const uint32_t ENCODER_DEBUG_INTERVAL_MS = 500; // Print every 500ms
    //drive_signed(30.0f, 30.0f); // Initial small movement to kick things off
    while (system_active) {
        printf("\n--- NEW LOOP ITERATION ---\n");
        if (!avoiding_obstacle) {
            // Phase 1: Line following while monitoring for obstacles
            float distance = ultrasonic_get_distance_cm();
            
            if (obstacle_detected(distance)) {
                printf("\n🚨 OBSTACLE DETECTED at %.1f cm! Stopping line following.\n", distance);
                all_stop();
                sleep_ms(1000);
                
                avoiding_obstacle = true;
                printf("🚨 Starting obstacle avoidance procedure...\n");
                
                // Execute complete avoidance cycle
                complete_avoidance_cycle();
                
                printf("🔄 Obstacle avoidance completed. Resuming line following...\n");
                avoiding_obstacle = false;
                
                // No need to reset PID variables - they're managed in PID_Line_Follow.c
            } else {
                // Continue line following
                follow_line_simple();
            }
        }
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        printf("Current time: %u ms\n", now);
        if (now - last_encoder_debug_time >= ENCODER_DEBUG_INTERVAL_MS) {
            if (!avoiding_obstacle) {
                debug_encoder_pulses("LINE_FOLLOW");
            } else {
                debug_encoder_pulses("AVOIDANCE");
            }
            last_encoder_debug_time = now;
        }

        
        printf(mqtt_is_connected() ? "MQTT connected\n" : "MQTT not connected\n");
        if (mqtt_is_connected() && (now - g_last_pub_ms >= 500)) {
            printf("MEOWMEOW\n");
            g_last_pub_ms = now;

            // Update speed and distance using new interrupt-based system
            update_speed_and_distance();

            // Get telemetry data - use the manually calculated speed which is correct
            speed = get_current_speed_cm_s();  // This now returns the correct 14.65 cm/s value
            dist  = get_total_distance_cm();
            ultra = ultrasonic_get_distance_cm();
            float direction = 0.0f;
            yaw = get_heading_fast(&direction);
            // float yaw   = 0.0f;   // TODO: wire in IMU yaw
            // Read IMU data for yaw
            printf("TRIPLE MEOW\n");

            // int16_t mx, my, mz;
            // if (read_mag_raw(&mx, &my, &mz)) {
            //     yaw = calculate_simple_yaw(mx, my);
            // } else {
            //     printf("[IMU] Magnetometer read failed\n");
            //     // Keep previous yaw value or use default
            // }
            
            printf("QUADRUPLE MEOW\n");
            
            // Debug output to confirm we're using the correct speed
            printf("PUBLISHING - Speed: %.2f cm/s, Distance: %.2f cm, Ultra: %.2f cm\n", 
                speed, dist, ultra);
            
            const char *state = "moving";
            mqtt_publish_telemetry(speed, dist, yaw, ultra, state);
        }

        // yield CPU (don't use tight sleep_ms in RTOS loops)
        vTaskDelay(pdMS_TO_TICKS(50));
            }
}

// ==============================
// TEST FUNCTIONS
// ==============================

void obstacle_avoidance_test(void) {
    printf("\n=== OBSTACLE AVOIDANCE TEST ===\n");
    
    ultrasonic_init();
    motor_encoder_init();
    servo_init();
    
    // Initialize the new interrupt-based encoder system
    speed_calc_init();
    
    imu_init();

    printf("Starting obstacle avoidance cycle...\n\n");
    complete_avoidance_cycle();
}

void calibrate_turning_pulses(void) {
    printf("\n=== TURNING CALIBRATION ===\n");
    printf("Press any key to start 90-degree right turn calibration...\n");
    getchar();
    
    turn_right_encoder_based(TURN_90_DEG_PULSES);
    
    printf("\nCalibration complete. Actual pulses used: %lu\n", get_average_pulses());
    printf("If the turn wasn't exactly 90 degrees, adjust TURN_90_DEG_PULSES in the code.\n");
}

// ==============================
// MAIN FUNCTION
// ==============================

static void robot_task(void *pv) {
    // optional: small delay to let USB come up
    vTaskDelay(pdMS_TO_TICKS(100));
    line_follow_with_obstacle_avoidance();   // your long-running loop lives here
    vTaskDelete(NULL);                       // never reached, but good practice
}

int main(void) {
    stdio_init_all();

    // Create the robot task (stack size can be tuned; 4096 words is a safe start)
    xTaskCreate(
        robot_task, 
        "robot",
        4096,                  // stack words (increase if needed)
        NULL,
        tskIDLE_PRIORITY + 2,  // priority
        NULL
    );

    // Start FreeRTOS
    vTaskStartScheduler();

    // Should never return
    while (1) { /* idle */ }
}