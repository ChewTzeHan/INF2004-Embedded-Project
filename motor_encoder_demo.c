/**
 * motor_encoder_demo.c - Motor control driver with PWM and encoder integration.
 */

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------- Pins (your wiring) ---------------- */
#define M1A 9
#define M1B 8
#define M2A 10
#define M2B 11

#define ENC1_DIG 6 // left / motor1 digital encoder
#define ENC2_DIG 4 // right / motor2 digital encoder

/* ========== Encoder accumulators ========== */
typedef struct {
    uint32_t pulse_count;
} enc_acc_t;

enc_acc_t E1 = {0}, E2 = {0};

/* ---------------- PWM helpers ---------------- */
void setup_pwm(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint     slice = pwm_gpio_to_slice_num(pin);
    uint32_t wrap  = 9999;
    float    div   = 125000000.0f / (20000.0f * (wrap + 1));
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

void set_pwm_pct(uint pin, float pct)
{
    if (pct < 0)
        pct = 0;
    if (pct > 100)
        pct = 100;
    uint     slice = pwm_gpio_to_slice_num(pin);
    uint32_t top   = pwm_hw->slice[slice].top;
    pwm_set_gpio_level(pin, (uint16_t)((pct / 100.0f) * (top + 1)));
}

void drive_signed(float left_pct, float right_pct)
{
    if (left_pct >= 0) {
        set_pwm_pct(M1A, left_pct);
        set_pwm_pct(M1B, 0);
    } else {
        set_pwm_pct(M1A, 0);
        set_pwm_pct(M1B, -left_pct);
    }
    if (right_pct >= 0) {
        set_pwm_pct(M2A, right_pct);
        set_pwm_pct(M2B, 0);
    } else {
        set_pwm_pct(M2A, 0);
        set_pwm_pct(M2B, -right_pct);
    }
}

void all_stop(void)
{
    drive_signed(0, 0);
}

/* ---------------- Polling-based encoder reading ---------------- */
void poll_encoders_during_test(uint32_t duration_ms)
{
    E1.pulse_count = 0;
    E2.pulse_count = 0;

    bool last_enc1 = gpio_get(ENC1_DIG);
    bool last_enc2 = gpio_get(ENC2_DIG);

    absolute_time_t start_time      = get_absolute_time();
    uint32_t        last_print_time = 0;

    while (absolute_time_diff_us(start_time, get_absolute_time()) < duration_ms * 1000) {
        bool current_enc1 = gpio_get(ENC1_DIG);
        bool current_enc2 = gpio_get(ENC2_DIG);

        // Detect rising edges
        if (current_enc1 && !last_enc1) {
            E1.pulse_count++;
        }
        if (current_enc2 && !last_enc2) {
            E2.pulse_count++;
        }

        last_enc1 = current_enc1;
        last_enc2 = current_enc2;

        // Print progress every 100ms
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 100) {
            printf("Time: %.1fs - Encoder counts: Left=%lu, Right=%lu\r",
                   (float)(current_time - to_ms_since_boot(start_time)) / 1000.0f, E1.pulse_count,
                   E2.pulse_count);
            last_print_time = current_time;
        }

        sleep_us(100); // Small delay to avoid overwhelming the CPU
    }
    printf("\n");
}

/* ---------------- Motor encoder initialization ---------------- */
void motor_encoder_init(void)
{
    // Initialize all PWM outputs for motors
    setup_pwm(M1A);
    setup_pwm(M1B);
    setup_pwm(M2A);
    setup_pwm(M2B);

    // Setup encoder inputs
    gpio_init(ENC1_DIG);
    gpio_set_dir(ENC1_DIG, GPIO_IN);
    gpio_pull_up(ENC1_DIG);

    gpio_init(ENC2_DIG);
    gpio_set_dir(ENC2_DIG, GPIO_IN);
    gpio_pull_up(ENC2_DIG);

    printf("Motor system initialized (Polling-based encoder reading)\n");
}

/* ---------------- Calibration function ---------------- */
void calibrate_motors(void)
{
    printf("\n=== MOTOR CALIBRATION ===\n");
    printf("Running both motors at 30%% speed for 2 seconds...\n\n");

    // Start both motors forward at 30% speed
    drive_signed(30.0f, 30.0f);

    // Run for 2 seconds while polling encoder pulses
    poll_encoders_during_test(2000);

    // Stop motors
    all_stop();

    // Get final counts
    uint32_t left_count  = E1.pulse_count;
    uint32_t right_count = E2.pulse_count;

    printf("\n=== CALIBRATION RESULTS ===\n");
    printf("Left motor encoder pulses:  %lu\n", left_count);
    printf("Right motor encoder pulses: %lu\n", right_count);

    // Calculate difference and compensation factors
    if (left_count == 0 || right_count == 0) {
        printf("ERROR: One or both encoders recorded 0 pulses!\n");
        printf("Check encoder wiring and connections.\n");
        return;
    }

    int32_t difference         = (int32_t)left_count - (int32_t)right_count;
    float   difference_percent = ((float)difference / ((left_count + right_count) / 2.0f)) * 100.0f;

    printf("Difference: %ld pulses (%.1f%%)\n", difference, difference_percent);

    // Calculate compensation factors
    float left_compensation  = 1.0f;
    float right_compensation = 1.0f;

    if (left_count > right_count) {
        // Left is faster, slow it down
        left_compensation = (float)right_count / (float)left_count;
        printf("Left motor is faster - applying compensation factor: %.3f\n", left_compensation);
    } else if (right_count > left_count) {
        // Right is faster, slow it down
        right_compensation = (float)left_count / (float)right_count;
        printf("Right motor is faster - applying compensation factor: %.3f\n", right_compensation);
    } else {
        printf("Motors are perfectly balanced! No compensation needed.\n");
    }

    // Calculate recommended speeds for straight movement
    float base_speed        = 30.0f;
    float recommended_left  = base_speed * left_compensation;
    float recommended_right = base_speed * right_compensation;

    printf("\n=== RECOMMENDED SETTINGS ===\n");
    printf("For straight movement at 30%% base speed:\n");
    printf("Left motor:  %.1f%%\n", recommended_left);
    printf("Right motor: %.1f%%\n", recommended_right);

    // Show the compensation factors to use in code
    printf("\nCompensation factors to use in your code:\n");
    printf("LEFT_COMPENSATION  = %.3f\n", left_compensation);
    printf("RIGHT_COMPENSATION = %.3f\n", right_compensation);
    printf("\nUsage example:\n");
    printf(
        "drive_signed(desired_speed * LEFT_COMPENSATION, desired_speed * RIGHT_COMPENSATION);\n");

    // Verification test
    printf("\n=== VERIFICATION TEST ===\n");
    printf("Testing compensation with calculated speeds for 1 second...\n");

    // Run with compensated speeds
    drive_signed(recommended_left, recommended_right);
    poll_encoders_during_test(1000);
    all_stop();

    printf("Verification results:\n");
    printf("Left encoder:  %lu pulses\n", E1.pulse_count);
    printf("Right encoder: %lu pulses\n", E2.pulse_count);

    int32_t new_difference = (int32_t)E1.pulse_count - (int32_t)E2.pulse_count;
    float   new_difference_percent =
        ((float)new_difference / ((E1.pulse_count + E2.pulse_count) / 2.0f)) * 100.0f;
    printf("New difference: %ld pulses (%.1f%%)\n", new_difference, new_difference_percent);

    if (abs(new_difference) < abs(difference) / 2) {
        printf("✓ Compensation successful! Motors are more balanced.\n");
    } else {
        printf("! Compensation may need further adjustment.\n");
    }
}
