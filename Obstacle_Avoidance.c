#include "Obstacle_Avoidance.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "ultrasonic.h"
#include "motor_encoder_demo.h"

// ==============================
// ENCODER-BASED TURNING CONFIG
// ==============================

// Configurable encoder pulse counts for turning
#define TURN_90_DEG_PULSES 8     // Pulses needed for 90-degree turn
#define TURN_180_DEG_PULSES 100  // Pulses needed for 180-degree turn
#define TURN_45_DEG_PULSES 25    // Pulses needed for 45-degree turn

// ==============================
// GLOBAL VARIABLES FOR ENCODER POLLING
// ==============================

volatile uint32_t encoder_left_count = 0;
volatile uint32_t encoder_right_count = 0;
volatile bool last_enc1_state = false;
volatile bool last_enc2_state = false;

// ==============================
// SERVO CONTROL FUNCTIONS
// ==============================

void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    // Configure PWM for 50Hz (20ms period) for standard servos
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f); // 125MHz / 125 = 1MHz
    pwm_config_set_wrap(&config, 20000);    // 1MHz / 20000 = 50Hz
    
    pwm_init(slice_num, &config, true);
    
    // Start at center position (90 degrees)
    servo_set_angle(85.0f);
    printf("[SERVO] Initialized on GPIO %d\n", SERVO_PIN);
}

void servo_set_angle(float angle) {
    // Constrain angle to valid range
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    
    // Convert angle to pulse width in microseconds
    float pulse_width_us = SERVO_MIN_PULSE_US + (angle / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    
    // Convert to PWM level (1MHz clock, so 1 count = 1us)
    uint16_t level = (uint16_t)pulse_width_us;
    
    pwm_set_gpio_level(SERVO_PIN, level);
}

// ==============================
// REAL-TIME ENCODER POLLING FUNCTIONS
// ==============================

void update_encoder_counts(void) {
    bool current_enc1 = gpio_get(ENC1_DIG);
    bool current_enc2 = gpio_get(ENC2_DIG);
    
    // Detect rising edges on left encoder (ENC1)
    if (current_enc1 && !last_enc1_state) {
        encoder_left_count++;
    }
    
    // Detect rising edges on right encoder (ENC2)
    if (current_enc2 && !last_enc2_state) {
        encoder_right_count++;
    }
    
    last_enc1_state = current_enc1;
    last_enc2_state = current_enc2;
}

void reset_encoder_counts(void) {
    encoder_left_count = 0;
    encoder_right_count = 0;
    last_enc1_state = gpio_get(ENC1_DIG);
    last_enc2_state = gpio_get(ENC2_DIG);
    printf("[ENCODER] Reset encoder counts\n");
}

uint32_t get_encoder_pulses(void) {
    // Return the average of both encoders for turning
    return (encoder_left_count + encoder_right_count) / 2;
}

void print_encoder_status(void) {
    printf("[ENCODER] Left: %lu, Right: %lu, Average: %lu\n", 
           encoder_left_count, encoder_right_count, get_encoder_pulses());
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
    float angle_C = fabsf(servo_angle_left - servo_angle_right) * (float)M_PI / 180.0f;
    
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
    
    const float SCAN_STEP = 5.0f; // Reduced resolution for faster scanning
    const float OBJECT_DETECTION_THRESHOLD = 30.0f;
    
    float max_left_distance = -1.0f;
    float max_right_distance = -1.0f;
    float max_left_angle = -1.0f;
    float max_right_angle = -1.0f;
    
    // Quick scan from 150° to 30°
    for (float angle = 150.0f; angle >= 30.0f; angle -= SCAN_STEP) {
        servo_set_angle(angle);
        sleep_ms(50);
        
        float distance = ultrasonic_get_distance_cm();
        
        if (distance > 2.0f && distance <= OBJECT_DETECTION_THRESHOLD) {
            if (angle > 90.0f) {
                if (distance > max_left_distance) {
                    max_left_distance = distance;
                    max_left_angle = angle;
                }
            } else {
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
// ENCODER-BASED TURNING FUNCTIONS
// ==============================

void turn_left_encoder_based(uint32_t target_pulses) {
    printf("[TURN] Turning LEFT for %lu encoder pulses\n", target_pulses);
    reset_encoder_counts();
    
    // Start turning left
    float turn_speed = 30.0f;
    drive_signed(-turn_speed, turn_speed);
    
    uint32_t last_print_time = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (get_encoder_pulses() < target_pulses) {
        // Update encoder counts in real-time
        update_encoder_counts();
        
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 100) {
            printf("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                   get_encoder_pulses(), target_pulses, current_time - start_time);
            last_print_time = current_time;
            
            // If no progress after 3 seconds, assume encoders aren't working
            if ((current_time - start_time) > 3000 && get_encoder_pulses() == 0) {
                printf("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                printf("[TURN] Falling back to timed turn\n");
                break;
            }
        }
        
        sleep_ms(10);
    }
    
    all_stop();
    printf("[TURN] Left turn completed: %lu pulses in %lums\n", 
           get_encoder_pulses(), to_ms_since_boot(get_absolute_time()) - start_time);
}

void turn_right_encoder_based(uint32_t target_pulses) {
    printf("[TURN] Turning RIGHT for %lu encoder pulses\n", target_pulses);
    reset_encoder_counts();
    
    // Start turning right
    float turn_speed = 30.0f;
    drive_signed(turn_speed, -turn_speed);
    
    uint32_t last_print_time = 0;
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while (get_encoder_pulses() < target_pulses) {
        // Update encoder counts in real-time
        update_encoder_counts();
        
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 100) {
            printf("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                   get_encoder_pulses(), target_pulses, current_time - start_time);
            last_print_time = current_time;
            
            // If no progress after 3 seconds, assume encoders aren't working
            if ((current_time - start_time) > 3000 && get_encoder_pulses() == 0) {
                printf("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
                printf("[TURN] Falling back to timed turn\n");
                break;
            }
        }
        
        sleep_ms(10);
    }
    
    all_stop();
    printf("[TURN] Right turn completed: %lu pulses in %lums\n", 
           get_encoder_pulses(), to_ms_since_boot(get_absolute_time()) - start_time);
}

// Convenience functions
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
// CHECK FOR OBJECT FUNCTION
// ==============================

bool check_if_object_still_there(void) {
    printf("[CHECK] Checking if object is still present...\n");
    
    // Turn right to face the object direction
    printf("[CHECK] Turning right to face object direction\n");
    turn_right_90_degrees();
    sleep_ms(500);
    
    // Check distance
    float distance = ultrasonic_get_distance_cm();
    printf("[CHECK] Distance after turn: %.1f cm\n", distance);
    
    bool object_present = obstacle_detected(distance);
    
    if (object_present) {
        printf("[CHECK] Object is still present at %.1f cm\n", distance);
        // Turn back to continue circling
        printf("[CHECK] Turning left to continue circling\n");
        turn_left_90_degrees();
    } else {
        printf("[CHECK] Object is no longer present\n");
        // Turn back to original position to continue pattern
        printf("[CHECK] Turning left to continue pattern\n");
        
    }
    
    return object_present;
}

// ==============================
// MODIFIED ENCIRCLING FUNCTION WITH CHECKING
// ==============================

void encircle_object_with_checking(void) {
    printf("[ENCIRCLE] Starting object circling with periodic checks\n");
    
    bool continue_circling = true;
    uint32_t check_counter = 0;
    uint32_t no_object_counter = 0;
    const uint32_t REQUIRED_NO_OBJECT_CHECKS = 2; // Need 2 consecutive checks with no object
    
    while (continue_circling) {
        check_counter++;
        printf("\n[ENCIRCLE] Check cycle %d (No-object count: %d/%d)\n", 
               check_counter, no_object_counter, REQUIRED_NO_OBJECT_CHECKS);
        
        // Phase 1: Move forward for 2.5 seconds alongside object
        printf("[ENCIRCLE] Moving forward for 2.5 seconds\n");
        drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
        sleep_ms(1750);
        all_stop();
        
        // Phase 2: Check if object is still there
        printf("[ENCIRCLE] Checking if object is still present\n");
        bool object_still_there = check_if_object_still_there();
        
        if (object_still_there) {
            // Reset no-object counter since we detected the object
            //no_object_counter = 0;
            printf("[ENCIRCLE] Object still detected, continuing to circle\n");
            printf("[ENCIRCLE] No-object counter reset to 0\n");
        } else {
            // Increment no-object counter
            no_object_counter++;
            printf("[ENCIRCLE] Object not detected - no-object counter: %d/%d\n", 
                   no_object_counter, REQUIRED_NO_OBJECT_CHECKS);
            
            if (no_object_counter >= REQUIRED_NO_OBJECT_CHECKS) {
                printf("[ENCIRCLE] ✅ Object cleared for %d consecutive checks!\n", REQUIRED_NO_OBJECT_CHECKS);
                printf("[ENCIRCLE] Moving straight forward to continue path\n");
                
                // Move straight forward for 3 seconds to continue past the object
                drive_signed(BASE_SPEED_LEFT, BASE_SPEED_RIGHT);
                sleep_ms(3000);
                all_stop();
                
                continue_circling = false;
                printf("[ENCIRCLE] Object avoidance completed successfully!\n");
            } else {
                printf("[ENCIRCLE] Object not detected, but need %d more check(s) to confirm\n", 
                       REQUIRED_NO_OBJECT_CHECKS - no_object_counter);
                printf("[ENCIRCLE] Continuing circling pattern...\n");
            }
        }
        
        // Safety check to avoid infinite loops
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
    
    // Final stop
    all_stop();
    printf("[ENCIRCLE] Object circling completed after %d checks\n", check_counter);
}

// ==============================
// SIMPLE ENCIRCLING FUNCTION (ORIGINAL)
// ==============================

void encircle_object_simple(void) {
    printf("[ENCIRCLE] Moving forward alongside object\n");
    
    bool circling_active = true;
    uint32_t no_object_counter = 0;
    const uint32_t MAX_NO_OBJECT_COUNT = 15;
    
    while (circling_active) {
        float distance = ultrasonic_get_distance_cm();
        
        if (distance < 0 || distance > OBSTACLE_DETECTION_DISTANCE_CM) {
            no_object_counter++;
            printf("[ENCIRCLE] No object detected (%d/%d)\n", no_object_counter, MAX_NO_OBJECT_COUNT);
            
            drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
            
            if (no_object_counter >= MAX_NO_OBJECT_COUNT) {
                printf("[ENCIRCLE] Object ended, stopping\n");
                all_stop();
                sleep_ms(1000);
                turn_right_90_degrees();
                circling_active = false;
            }
        } 
        else {
            no_object_counter = 0;
            drive_signed(CIRCLE_BASE_SPEED_LEFT, CIRCLE_BASE_SPEED_RIGHT);
            printf("[ENCIRCLE] Moving forward - Distance: %.1fcm\n", distance);
        }
        
        sleep_ms(50);
    }
}

// ==============================
// COMPLETE AVOIDANCE CYCLE
// ==============================

void complete_avoidance_cycle(void) {
    printf("[CYCLE] Starting complete avoidance cycle\n");
    
    bool cycle_active = true;
    
    while (cycle_active) {
        printf("\n=== NEW CYCLE STARTING ===\n");
        
        // Phase 1: Approach object
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
                
                // Quick scan
                printf("[CYCLE] Scanning object width\n");
                ObjectScanResult scan_result = scan_object_width();
                
                if (scan_result.width_cm > 0) {
                    printf("[CYCLE] Object width: %.1f cm\n", scan_result.width_cm);
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
        
        // Phase 2: Turn left to start circling
        printf("[CYCLE] Phase 2: Turning left to start circling\n");
        sleep_ms(1000);
        turn_left_90_degrees();
        
        // Phase 3: Circle object with periodic checks
        printf("[CYCLE] Phase 3: Circling object with periodic checks\n");
        encircle_object_with_checking();
        
        printf("[CYCLE] Object avoidance completed!\n");
        printf("[CYCLE] Robot has successfully navigated around the object.\n");
        printf("[CYCLE] Stopping for now. Restart program to begin new cycle.\n");
        
        // Stop the cycle after completing one full object avoidance
        cycle_active = false;
        
        // Reset servo to center position
        servo_set_angle(85.0f);
        sleep_ms(500);
    }
}

// ==============================
// TEST FUNCTION
// ==============================

void obstacle_avoidance_test(void) {
    printf("\n=== OBSTACLE AVOIDANCE TEST ===\n");
    printf("Initializing systems...\n");
    
    ultrasonic_init();
    motor_encoder_init();
    servo_init();
    
    // Initialize encoder polling
    reset_encoder_counts();
    
    printf("Encoder turning configuration:\n");
    printf("90-degree turn: %d pulses\n", TURN_90_DEG_PULSES);
    printf("45-degree turn: %d pulses\n", TURN_45_DEG_PULSES);
    printf("Behavior: Requires 2 consecutive no-object checks before completing\n");
    printf("Starting obstacle avoidance cycle...\n\n");
    
    complete_avoidance_cycle();
}

// ==============================
// CALIBRATION FUNCTION
// ==============================

void calibrate_turning_pulses(void) {
    printf("\n=== TURNING CALIBRATION ===\n");
    printf("This will help you find the right pulse counts for turning\n");
    printf("Press any key to start 90-degree right turn calibration...\n");
    getchar();
    
    turn_right_encoder_based(TURN_90_DEG_PULSES);
    
    printf("\nCalibration complete. Actual pulses used: %lu\n", get_encoder_pulses());
    printf("If the turn wasn't exactly 90 degrees, adjust TURN_90_DEG_PULSES in the code.\n");
}

// ==============================
// MAIN FUNCTION
// ==============================

// int main(void) {
//     stdio_init_all();
//     sleep_ms(4000);
    
//     printf("\n=== Obstacle Avoidance System ===\n");
//     printf("Behavior: Approach -> Turn -> Move 2.5s -> Check -> Repeat until 2x no-object -> Move forward\n");
    
//     // Uncomment to run calibration first
//     // calibrate_turning_pulses();
    
//     obstacle_avoidance_test();
    
//     return 0;
// }