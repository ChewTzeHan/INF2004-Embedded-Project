#include "Obstacle_Avoidance.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "log.h"
#include "ultrasonic.h"
#include "motor_encoder_demo.h"
#include "ir_sensor.h"
#include "PID_Line_Follow.h"
#include "ir_sensor.h"
#include "mqtt_client.h"
#include "encoder.h"  // Wheel encoder interface for speed and distance calculation

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"

#include "imu_raw_demo.h"

// ==============================
// PID LINE FOLLOWING CONSTANTS
// ==============================

#define TURN_90_DEG_PULSES  9  // Adjust based on your encoder counts
#define TURN_180_DEG_PULSES  16 // Adjust based on your encoder counts
#define TURN_45_DEG_PULSES   4  // Adjust based on your encoder counts

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
    LOG_INFO("[SPEED_CALC] Initialized with interrupt-based encoders and manual speed calculation\n");
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
    LOG_INFO("SPEED CALC: dist_traveled=%.2fcm, time=%.3fs, speed=%.2fcm/s\n", 
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
    LOG_INFO("[SPEED_CALC] Total distance reset using interrupt-based encoders\n");
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
    LOG_INFO("[SERVO] Initialized on GPIO %d\n", SERVO_PIN);
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
    
    LOG_INFO("[ENCODER] Left: %ld pulses, %.1f cm, %.1f cm/s | Right: %ld pulses, %.1f cm, %.1f cm/s\n", 
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
    float angle_C = fabsf((servo_angle_left+15.0f) - (servo_angle_right-15.0f)) * (float)M_PI / 180.0f;
    
    float a_squared = distance_left * distance_left;
    float b_squared = distance_right * distance_right;
    float two_ab_cosC = 2.0f * distance_left * distance_right * cosf(angle_C);
    
    float c_squared = a_squared + b_squared - two_ab_cosC;
    
    if (c_squared < 0) {
        c_squared = fabsf(c_squared);
    }
    
    float width = sqrtf(c_squared);
    
    LOG_INFO("[WIDTH CALC] Left: %.1fcm @ %.1f deg, Right: %.1fcm @ %.1f deg, Width: %.1fcm\n",
           distance_left, servo_angle_left, distance_right, servo_angle_right, width);
    
    return width;
}

ObjectScanResult scan_object_width(void) {
    LOG_INFO("[SCAN] Starting object width scan\n");
    
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
        LOG_INFO("[SCAN] Angle: %.1f deg, Distance: %.1f cm\n", angle, distance);
        
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
    LOG_INFO("[TURN] Turning LEFT for %lu encoder pulses\n", target_pulses);
    
    // Reset encoder counts using new system
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    
    float turn_speed = 50.0f;
    drive_signed(-turn_speed*1.25f, turn_speed);
    
    // Wait until we reach the target pulse count
    uint32_t last_print_time = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (get_average_pulses() < target_pulses) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 100) {
            LOG_INFO("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                get_average_pulses(), target_pulses, current_time - start_time);
            last_print_time = current_time;
            
            // Safety timeout - if no encoder pulses detected after 3 seconds, break out
            if ((current_time - start_time) > 3000 && get_average_pulses() == 0) {
                LOG_INFO("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                LOG_INFO("[TURN] Falling back to timed turn\n");
                break;
            }
        }
        sleep_ms(10);
    }
    
    all_stop();
    LOG_INFO("[TURN] Left turn completed: %lu pulses achieved\n", get_average_pulses());
}

void turn_right_encoder_based(uint32_t target_pulses) {
    LOG_INFO("[TURN] Turning RIGHT for %lu encoder pulses\n", target_pulses);
    
    // Reset encoder counts using new system
    encoder_reset_distance(ENCODER_LEFT_GPIO);
    encoder_reset_distance(ENCODER_RIGHT_GPIO);
    
    float turn_speed = 50.0f;
    drive_signed(turn_speed*1.25f, -turn_speed);
    
    // Wait until we reach the target pulse count
    uint32_t last_print_time = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (get_average_pulses() < target_pulses) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 100) {
            LOG_INFO("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                get_average_pulses(), target_pulses, current_time - start_time);
            last_print_time = current_time;
            
            // Safety timeout - if no encoder pulses detected after 3 seconds, break out
            if ((current_time - start_time) > 3000 && get_average_pulses() == 0) {
                LOG_INFO("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                LOG_INFO("[TURN] Falling back to timed turn\n");
                break;
            }
        }
        sleep_ms(10);
    }
    
    all_stop();
    LOG_INFO("[TURN] Right turn completed: %lu pulses achieved\n", get_average_pulses());
}

void turn_left_90_degrees(void) {
    turn_left_encoder_based(TURN_90_DEG_PULSES);
    // drive_signed(-40*1.25f, 40);
    // sleep_ms(1250);
    // all_stop();
}

void turn_right_90_degrees(void) {
    // drive_signed(40*1.25f, -40);
    // sleep_ms(1250);
    // all_stop();
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
    turn_right_90_degrees();
    LOG_INFO("[CHECK] Scanning 90-degree arc to check if object is still present...\n");
    
    const float SCAN_START_ANGLE = 45.0f;   // Start scanning at 45 degrees
    const float SCAN_END_ANGLE = 135.0f;    // End scanning at 135 degrees  
    const float SCAN_STEP = 2.0f;          // 5 degree steps
    const float OBJECT_DETECTION_THRESHOLD = OBSTACLE_DETECTION_DISTANCE_CM;
    
    bool object_detected = false;
    int detection_count = 0;
    
    LOG_INFO("[CHECK] Scanning from %.1f deg to %.1f deg in %.1f deg steps\n", 
           SCAN_START_ANGLE, SCAN_END_ANGLE, SCAN_STEP);
    
    // Scan from left to right (45 deg to 135 deg)
    for (float angle = SCAN_START_ANGLE; angle <= SCAN_END_ANGLE; angle += SCAN_STEP) {
        servo_set_angle(angle);
        sleep_ms(100);  // Allow servo to settle
        
        float distance = ultrasonic_get_distance_cm();
        LOG_INFO("[CHECK] Angle: %.1f deg, Distance: %.1f cm - ", angle, distance);
        
        if (obstacle_detected(distance)) {
            LOG_INFO("OBJECT DETECTED\n");
            object_detected = true;
            detection_count++;
            break;

        } else {
            LOG_INFO("clear\n");
        }
        
        sleep_ms(50);  // Brief pause between readings
    }
    
    // Return servo to center position
    servo_set_angle(85.0f);
    sleep_ms(200);
    
    LOG_INFO("[CHECK] Scan complete: %d detections in %d positions\n", 
           detection_count, (int)((SCAN_END_ANGLE - SCAN_START_ANGLE) / SCAN_STEP) + 1);
    
    if (object_detected) {
        LOG_INFO("[CHECK] Object is still present (%d detections)\n", detection_count);
    } else {
        LOG_INFO("[CHECK] Object is no longer present\n");
    }
    
    return object_detected;
}

// ==============================
// ENCIRCLING FUNCTION WITH CHECKING - UPDATED TO USE NEW ENCODER SYSTEM
// ==============================

void encircle_object_with_checking(void) {
    LOG_INFO("[ENCIRCLE] Starting object circling with periodic checks\n");
    
    bool continue_circling = true;
    uint32_t check_counter = 0;
    uint32_t no_object_counter = 0;
    const uint32_t REQUIRED_NO_OBJECT_CHECKS = 2;
    uint8_t turns_finished = 0;
    while (continue_circling) {
        check_counter++;
        LOG_INFO("\n[ENCIRCLE] Check cycle %d (No-object count: %d/%d)\n", 
               check_counter, no_object_counter, REQUIRED_NO_OBJECT_CHECKS);
        
        LOG_INFO("[ENCIRCLE] Moving forward for 2.5 seconds\n");
        encoder_reset_distance(ENCODER_LEFT_GPIO);
        encoder_reset_distance(ENCODER_RIGHT_GPIO);
        drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
        //sleep_ms(500);

        uint32_t last_print_time = 0;
        uint32_t start_time = to_ms_since_boot(get_absolute_time());
        
        #define target_pulses 12
        while (get_average_pulses() < target_pulses) {
            // No need to call update_encoder_counts() - interrupts handle it automatically
            
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - last_print_time >= 100) {
                LOG_INFO("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                    get_average_pulses(), target_pulses, current_time - start_time);
                last_print_time = current_time;
                
                if ((current_time - start_time) > 3000 && get_average_pulses() == 0) {
                    LOG_INFO("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                    LOG_INFO("[TURN] Falling back to timed turn\n");
                    break;
                }
            }

            if(turns_finished >= 2){
                uint16_t ir_raw = ir_read_raw();
                int color_state = classify_colour(ir_raw);

                if (color_state == 1) {
                    LOG_INFO("[ENCIRCLE] BLACK LINE DETECTED DURING CIRCLE! Raw: %u, Stopping.\n", ir_raw);
                    continue_circling = false;
                    break;
                }
            }

            
            sleep_ms(10);
        }
        
        all_stop();
        sleep_ms(2000);

        if(!continue_circling){
            break;
        }
        
        LOG_INFO("[ENCIRCLE] Checking if object is still present\n");
        bool object_still_there = check_if_object_still_there();
        
        if (object_still_there) {
            LOG_INFO("[ENCIRCLE] Object still detected, continuing to circle\n");
            LOG_INFO("[ENCIRCLE] No-object counter reset to 0\n");
            no_object_counter = 0;
            turn_left_90_degrees();
        } else {
            no_object_counter++;
            LOG_INFO("[ENCIRCLE] Object not detected - no-object counter: %d/%d\n", 
                   no_object_counter, REQUIRED_NO_OBJECT_CHECKS);
            
            if (no_object_counter >= REQUIRED_NO_OBJECT_CHECKS && turns_finished < 2) {
                LOG_INFO("[ENCIRCLE] Object cleared for %d consecutive checks!\n", REQUIRED_NO_OBJECT_CHECKS);
                LOG_INFO("[ENCIRCLE] Moving straight forward to continue path\n");

                drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
                while (get_average_pulses() < target_pulses) {
                    // No need to call update_encoder_counts() - interrupts handle it automatically
                    
                    uint32_t current_time = to_ms_since_boot(get_absolute_time());
                    if (current_time - last_print_time >= 100) {
                        LOG_INFO("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                            get_average_pulses(), target_pulses, current_time - start_time);
                        last_print_time = current_time;
                        
                        if ((current_time - start_time) > 3000 && get_average_pulses() == 0) {
                            LOG_INFO("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                            LOG_INFO("[TURN] Falling back to timed turn\n");
                            break;
                        }
                    }
                    
                    sleep_ms(10);
                }

                all_stop();
                no_object_counter = 0;
                turns_finished++;

            all_stop();
                
                //continue_circling = false;
                LOG_INFO("[ENCIRCLE] Object avoidance completed successfully!\n");
            } 
            else {
                LOG_INFO("[ENCIRCLE] Object not detected, but need %d more check(s) to confirm\n", 
                       REQUIRED_NO_OBJECT_CHECKS - no_object_counter);
                LOG_INFO("[ENCIRCLE] Continuing circling pattern...\n");
                turn_left_90_degrees();
            }
        }
        
        if (check_counter >= 15) {
            LOG_INFO("[ENCIRCLE] Safety limit reached (15 checks)\n");
            LOG_INFO("[ENCIRCLE] Moving straight forward to break possible loop\n");
            drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
            sleep_ms(3000);
            all_stop();
            continue_circling = false;
        }

        
        
        sleep_ms(500);
    }
    
    all_stop();
    LOG_INFO("[ENCIRCLE] Object circling completed after %d checks\n", check_counter);
}

// ==============================
// COMPLETE AVOIDANCE CYCLE
// ==============================
float yaw = 0.0f;
float speed = 0.0f;
float dist = 0.0f;
float ultra = 0.0f;
void complete_avoidance_cycle(void) {
    LOG_INFO("[CYCLE] Starting complete avoidance cycle\n");
    
    bool cycle_active = true;
    
    while (cycle_active) {
        LOG_INFO("\n=== NEW CYCLE STARTING ===\n");
        
        LOG_INFO("[CYCLE] Phase 1: Approaching object\n");
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
                LOG_INFO("[CYCLE] Object at stop distance (%.1f cm)! Stopping\n", distance);
                all_stop();
                approach_active = false;
                object_initially_detected = true;
                
                LOG_INFO("[CYCLE] Scanning object width\n");
                mqtt_publish_telemetry(speed, dist, yaw, ultra, "scanning object");
                ObjectScanResult scan_result = scan_object_width();
                
                if (scan_result.width_cm > 0) {
                    LOG_INFO("[CYCLE] Object width: %.1f cm\n", scan_result.width_cm);
                    char width_str[30];
                    snLOG_INFO(width_str, sizeof(width_str), "object width: %.1fcm", scan_result.width_cm);
                    mqtt_publish_telemetry(speed, dist, yaw, ultra, width_str);
                    sleep_ms(3000);
            
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
        
        LOG_INFO("[CYCLE] Phase 2: Turning left to start circling\n");
        sleep_ms(1000);
        turn_left_90_degrees();
        sleep_ms(1000);
        
        LOG_INFO("[CYCLE] Phase 3: Circling object with periodic checks\n");
        encircle_object_with_checking();
        
        // LOG_INFO("[CYCLE] Object avoidance completed!\n");
        // LOG_INFO("[CYCLE] Robot has successfully navigated around the object.\n");
        // LOG_INFO("[CYCLE] Stopping for now. Restart program to begin new cycle.\n");
        
        cycle_active = false;
        servo_set_angle(85.0f);

        sleep_ms(2000);
        drive_signed(40, 40);
        sleep_ms(500);
        drive_signed(-40, 60); //right turn
        sleep_ms(500);
        all_stop();
        sleep_ms(2000);
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
    
    LOG_INFO("[ENCODER_DEBUG][%s] Time: %lums\n", context, current_time);
    LOG_INFO("  Left: %ld (+%ld) pulses, %.1f cm, %.1f cm/s\n", 
           left_count, left_diff, left_distance, left_speed);
    LOG_INFO("  Right: %ld (+%ld) pulses, %.1f cm, %.1f cm/s\n", 
           right_count, right_diff, right_distance, right_speed);
    LOG_INFO("  Average: %ld pulses, %.1f cm, %.1f cm/s\n",
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
    LOG_INFO("\n=== COMBINED LINE FOLLOWING WITH OBSTACLE AVOIDANCE ===\n");
    LOG_INFO("Behavior: Line follow until object detected -> Avoid object -> Resume line following\n");
    
    bool system_active = true;
    

    // Start Wi-Fi + MQTT (safe: it locks around lwIP calls inside)
    bool connected = false;
    int attempts = 0;

    if (!wifi_and_mqtt_start()) {
        LOG_INFO("[NET] Wi-Fi/MQTT start failed (continuing without MQTT)\n");
    }

    // if (!connected) {
    //     LOG_INFO("[NET] Wi-Fi/MQTT start failed after %d attempts (continuing without MQTT)\n", 
    //            MAX_CONNECTION_ATTEMPTS);
    // } else {
    //     LOG_INFO("[NET] Wi-Fi/MQTT connected successfully\n");
    // }
    
    // LOG_INFO("[NET] Local IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    
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
    LOG_INFO("Starting combined operation...\n");


    // while(1){
    //     turn_left_90_degrees();
    //     sleep_ms(1000);
    //     turn_right_90_degrees();
    //     sleep_ms(1000);
    // }


    uint32_t last_encoder_debug_time = 0;
    const uint32_t ENCODER_DEBUG_INTERVAL_MS = 500; // Print every 500ms
    bool avoiding_obstacle = false;
    float distance = 0.0f;
    //drive_signed(30.0f, 30.0f); // Initial small movement to kick things off
    while (system_active) {
        LOG_INFO("\n--- NEW LOOP ITERATION ---\n");
        if (!avoiding_obstacle) {
            LOG_INFO("sensing...\n");
            // Phase 1: Line following while monitoring for obstacles
            distance = ultrasonic_get_distance_cm();
            LOG_INFO("distance gained: %.1f cm\n", distance);
            if (obstacle_detected(distance)) {
                LOG_INFO("\n[ALERT] OBSTACLE DETECTED at %.1f cm! Stopping line following.\n", distance);
                all_stop();
                sleep_ms(1000);
                
                avoiding_obstacle = true;
                LOG_INFO("[ALERT] Starting obstacle avoidance procedure...\n");
                
                // Execute complete avoidance cycle
                complete_avoidance_cycle();
                
                LOG_INFO("[STATE] Obstacle avoidance completed. Resuming line following...\n");
                avoiding_obstacle = false;
                
                // No need to reset PID variables - they're managed in PID_Line_Follow.c
            } else {
                // Continue line following
                follow_line_simple();
            }
        }
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        LOG_INFO("Current time: %u ms\n", now);
        if (now - last_encoder_debug_time >= ENCODER_DEBUG_INTERVAL_MS) {
            if (!avoiding_obstacle) {
                debug_encoder_pulses("LINE_FOLLOW");
            } else {
                debug_encoder_pulses("AVOIDANCE");
            }
            last_encoder_debug_time = now;
        }

        
        //LOG_INFO(mqtt_is_connected() ? "MQTT connected\n" : "MQTT not connected\n");
        if (mqtt_is_connected() && (now - g_last_pub_ms >= 500)) {
            g_last_pub_ms = now;

            // Update speed and distance using new interrupt-based system
            update_speed_and_distance();

            // Get telemetry data - use the manually calculated speed which is correct
            speed = get_current_speed_cm_s();  // This now returns the correct 14.65 cm/s value
            dist  = get_total_distance_cm();
            ultra = ultrasonic_get_distance_cm();
            float direction = 0.0f;
            yaw = get_heading_fast(&direction);

            // Debug output to confirm we're using the correct speed
            LOG_INFO("PUBLISHING - Speed: %.2f cm/s, Distance: %.2f cm, Ultra: %.2f cm\n", 
                speed, dist, ultra);
            
            const char *state = "moving";
            mqtt_publish_telemetry(speed, dist, yaw, ultra, state);
        }

        // yield CPU (don't use tight sleep_ms in RTOS loops)
        vTaskDelay(pdMS_TO_TICKS(20));
            }
}

// Modified obstacle avoidance without built-in line following
// Add to Obstacle_Avoidance.c
bool check_obstacle_detection(void) {
    // Use the fast detection for the main loop
    return ultrasonic_detect_obstacle_fast();
}

void avoid_obstacle_only(void) {
    LOG_INFO("[AVOID] Starting obstacle avoidance procedure...\n");
    
    // Execute the core avoidance cycle without the line following wrapper
    bool cycle_active = true;
    
    while (cycle_active) {
        LOG_INFO("\n=== OBSTACLE AVOIDANCE CYCLE ===\n");
        
        LOG_INFO("[AVOID] Phase 1: Object width scanning\n");
        ObjectScanResult scan_result = scan_object_width();
        
        if (scan_result.width_cm > 0) {
            LOG_INFO("[AVOID] Object width: %.1f cm\n", scan_result.width_cm);
            
            // Send telemetry with obstacle width information
            char width_str[100];
            snLOG_INFO(width_str, sizeof(width_str), 
                    "OBSTACLE_WIDTH: %.1fcm", 
                    scan_result.width_cm);
            
            // Get current sensor values for telemetry
            float speed = get_current_speed_cm_s();
            float distance = get_total_distance_cm();
            float ultra = ultrasonic_get_distance_cm();
            float direction = 0.0f;
            float yaw = get_heading_fast(&direction);
            
            mqtt_publish_telemetry(speed, distance, yaw, ultra, width_str);
            LOG_INFO("[AVOID] Obstacle width telemetry sent: %s\n", width_str);
            
            sleep_ms(2000); // Give time to see the telemetry
            mqtt_publish_telemetry(speed, distance, yaw, ultra, "Performing Object Avoidance");
        }
        
        LOG_INFO("[AVOID] Phase 2: Turning left to start circling\n");

        turn_left_90_degrees();
        sleep_ms(1000);

        LOG_INFO("[AVOID] Phase 3: Circling object\n");
        encircle_object_with_checking();
        
        cycle_active = false;
        servo_set_angle(85.0f);

        sleep_ms(2000);
        drive_signed(40*1.25f, 40);
        sleep_ms(700);
        drive_signed(-40, 60); //right turn
        sleep_ms(400);
        all_stop();
        sleep_ms(2000);
    }
}