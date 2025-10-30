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

#include "FreeRTOS.h"
#include "task.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"


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

volatile uint32_t encoder_left_count = 0;
volatile uint32_t encoder_right_count = 0;
volatile bool last_enc1_state = false;
volatile bool last_enc2_state = false;

static volatile uint32_t g_last_pub_ms = 0;

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
// REAL-TIME ENCODER POLLING FUNCTIONS
// ==============================

void update_encoder_counts(void) {
    bool current_enc1 = gpio_get(ENC1_DIG);
    bool current_enc2 = gpio_get(ENC2_DIG);
    
    if (current_enc1 && !last_enc1_state) {
        encoder_left_count++;
    }
    
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
    
    const float SCAN_STEP = 5.0f;
    const float OBJECT_DETECTION_THRESHOLD = 30.0f;
    
    float max_left_distance = -1.0f;
    float max_right_distance = -1.0f;
    float max_left_angle = -1.0f;
    float max_right_angle = -1.0f;
    
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
#define sleep_duration_ms 500
void turn_left_encoder_based(uint32_t target_pulses) {
    printf("[TURN] Turning LEFT for %lu encoder pulses\n", target_pulses);
    reset_encoder_counts();
    
    float turn_speed = 50.0f;
    drive_signed(-turn_speed, turn_speed);
    sleep_ms(sleep_duration_ms);

    

    // uint32_t last_print_time = 0;
    // uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    // while (get_encoder_pulses() < target_pulses) {
    //     update_encoder_counts();
        
    //     uint32_t current_time = to_ms_since_boot(get_absolute_time());
    //     if (current_time - last_print_time >= 100) {
    //         printf("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
    //                get_encoder_pulses(), target_pulses, current_time - start_time);
    //         last_print_time = current_time;
            
    //         if ((current_time - start_time) > 3000 && get_encoder_pulses() == 0) {
    //             printf("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
    //             printf("[TURN] Falling back to timed turn\n");
    //             break;
    //         }
    //     }
        
    //     sleep_ms(10);
    // }
    
    all_stop();
    // printf("[TURN] Left turn completed: %lu pulses in %lums\n", 
    //        get_encoder_pulses(), to_ms_since_boot(get_absolute_time()) - start_time);
}

void turn_right_encoder_based(uint32_t target_pulses) {
    printf("[TURN] Turning RIGHT for %lu encoder pulses\n", target_pulses);
    reset_encoder_counts();
    
    float turn_speed = 50.0f;
    drive_signed(turn_speed, -turn_speed);
    sleep_ms(sleep_duration_ms);
    // uint32_t last_print_time = 0;
    // uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    // while (get_encoder_pulses() < target_pulses) {
    //     update_encoder_counts();
        
    //     uint32_t current_time = to_ms_since_boot(get_absolute_time());
    //     if (current_time - last_print_time >= 100) {
    //         printf("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
    //                get_encoder_pulses(), target_pulses, current_time - start_time);
    //         last_print_time = current_time;
            
    //         if ((current_time - start_time) > 3000 && get_encoder_pulses() == 0) {
    //             printf("[TURN] WARNING: No encoder pulses detected after 3 seconds!\n");
    //             printf("[TURN] Falling back to timed turn\n");
    //             break;
    //         }
    //     }
        
    //     sleep_ms(10);
    // }
    
    all_stop();
    // printf("[TURN] Right turn completed: %lu pulses in %lums\n", 
    //        get_encoder_pulses(), to_ms_since_boot(get_absolute_time()) - start_time);
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
// ENCIRCLING FUNCTION WITH CHECKING
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
        while (get_encoder_pulses() < target_pulses) {
            update_encoder_counts();
            
            uint32_t current_time = to_ms_since_boot(get_absolute_time());
            if (current_time - last_print_time >= 100) {
                printf("[TURN] Progress: %lu/%lu pulses (Time: %lums)\n", 
                    get_encoder_pulses(), target_pulses, current_time - start_time);
                last_print_time = current_time;
                
                if ((current_time - start_time) > 3000 && get_encoder_pulses() == 0) {
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
        
        printf("[CYCLE] Phase 2: Turning left to start circling\n");
        sleep_ms(1000);
        turn_left_90_degrees();
        sleep_ms(1000);
        
        printf("[CYCLE] Phase 3: Circling object with periodic checks\n");
        encircle_object_with_checking();
        
        printf("[CYCLE] Object avoidance completed!\n");
        printf("[CYCLE] Robot has successfully navigated around the object.\n");
        printf("[CYCLE] Stopping for now. Restart program to begin new cycle.\n");
        
        cycle_active = false;
        servo_set_angle(85.0f);
        sleep_ms(500);
    }
}

// ==============================
// COMBINED LINE FOLLOWING WITH OBSTACLE AVOIDANCE
// ==============================
// ==============================
// COMBINED LINE FOLLOWING WITH OBSTACLE AVOIDANCE
// ==============================

void line_follow_with_obstacle_avoidance(void) {
    printf("\n=== COMBINED LINE FOLLOWING WITH OBSTACLE AVOIDANCE ===\n");
    printf("Behavior: Line follow until object detected -> Avoid object -> Resume line following\n");
    
    bool system_active = true;
    bool avoiding_obstacle = false;

    // Start Wi-Fi + MQTT (safe: it locks around lwIP calls inside)
    if (!wifi_and_mqtt_start()) {
        printf("[NET] Wi-Fi/MQTT start failed (continuing without MQTT)\n");
    }
    
    printf("[NET] Local IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_default)));
    
    // Initialize systems
    ultrasonic_init();
    motor_encoder_init();
    servo_init();
    ir_init(NULL);
    reset_encoder_counts();
    
    printf("Starting combined operation...\n");

    uint32_t last_pub = to_ms_since_boot(get_absolute_time());
    
    while (system_active) {
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
                //complete_avoidance_cycle();
                
                printf("🔄 Obstacle avoidance completed. Resuming line following...\n");
                avoiding_obstacle = false;
                
                // No need to reset PID variables - they're managed in PID_Line_Follow.c
            } else {
                // Continue line following
                //follow_line_simple();
            }
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());
        printf(mqtt_is_connected() ? "MQTT connected\n" : "MQTT not connected\n");
        if (mqtt_is_connected() && (now - g_last_pub_ms >= 500)) {
            printf("MEOWMEOW\n");
            g_last_pub_ms = now;

            float ultra = ultrasonic_get_distance_cm();
            float speed = 0.0f;   // TODO: derive from encoders
            float dist  = 0.0f;   // TODO: track distance if you want
            float yaw   = 0.0f;   // TODO: wire in IMU yaw

            const char *state = obstacle_detected(ultra) ? "obstacle" : "moving";
            mqtt_publish_telemetry(speed, dist, yaw, ultra, state);
        }

        mqtt_publish_telemetry(40.0f, 6.0f, 3.0f, 0.21f, "line_following");
        // yield CPU (don’t use tight sleep_ms in RTOS loops)
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
    reset_encoder_counts();
    
    printf("Starting obstacle avoidance cycle...\n\n");
    complete_avoidance_cycle();
}

void calibrate_turning_pulses(void) {
    printf("\n=== TURNING CALIBRATION ===\n");
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
//     printf("=== Obstacle Avoidance Main Program ===\n");
//     // Wi-Fi + MQTT
//     if (!wifi_and_mqtt_start()) {
//         printf("[NET] Wi-Fi/MQTT start failed\n");
//         // carry on with line following if you want, or return
//     }
    
//     printf("\n=== Combined Line Following & Obstacle Avoidance ===\n");
//     printf("Behavior: Line follow -> Detect obstacle -> Avoid -> Resume line following\n");
    
//     // Uncomment for calibration
//     // calibrate_turning_pulses();
    
//     // Run the combined system
//     line_follow_with_obstacle_avoidance();
    
//     return 0;
// }

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
        //line_follow_with_obstacle_avoidance,
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
