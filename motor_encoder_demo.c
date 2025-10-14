#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "FreeRTOS.h"
#include "task.h"

/* ---------------- Pins (your wiring) ---------------- */
#define M1A 8
#define M1B 9
#define M2A 10
#define M2B 11

#define ENC1_DIG 6   // left / motor1 digital encoder
#define ENC2_DIG 4   // right / motor2 digital encoder

#define IR_ADC_PIN 26  // optional analog (ADC0) – not used in math

/* ---------------- PWM config ---------------- */
#define F_PWM_HZ     20000.0f
#define SYS_CLK_HZ 125000000u

/* ---------------- Encoder math (optional RPM) ---------------- */
#define TICKS_PER_REV 20.0f   // set to your encoder ticks/rev if known, else leave 20
#define GEAR_RATIO     1.0f   // motor:wheel ratio if encoder on motor shaft

/* ========== Encoder accumulators (for polling) ========== */
typedef struct {
    volatile uint32_t last_rise;  // Time of last rising edge
    volatile uint32_t last_fall;  // Time of last falling edge
    // per-move accumulators:
    volatile uint32_t period_sum; // Total sum of all measured periods
    volatile uint32_t high_sum;   // Total sum of all measured high times
    volatile uint32_t edges;      // number of valid periods added
    volatile uint32_t highs;      // number of valid highs added
    uint32_t last_state;          // Last GPIO state for edge detection
} enc_acc_t;

// One struct for each encoder - now accessible from main.c
enc_acc_t E1 = {0}, E2 = {0};

// Helper to get current time in microseconds
static inline uint32_t now_us(void){ return time_us_32(); }

/* --------------------------------------------------------------------------
   ENCODER POLLING FUNCTION
   Called periodically to check for encoder state changes
   -------------------------------------------------------------------------- */
static void enc_poll_common(enc_acc_t* E, bool current_state) {
    uint32_t t = now_us();
    
    // Detect rising edge (0 -> 1)
    if (current_state && !E->last_state) {
        if (E->last_rise) {
            uint32_t per = t - E->last_rise;
            // guard out crazy values (<50 us or >1 s)
            if (per > 50 && per < 1000000) { 
                E->period_sum += per; 
                E->edges++; 
            }
        }
        E->last_rise = t;
    }
    
    // Detect falling edge (1 -> 0)
    if (!current_state && E->last_state) {
        E->last_fall = t;
        // Calculate how long signal stayed high (fall - rise)
        if (E->last_fall > E->last_rise) {
            uint32_t hi = E->last_fall - E->last_rise;
            if (hi > 20 && hi < 1000000) { 
                E->high_sum += hi; 
                E->highs++; 
            }
        }
    }
    
    E->last_state = current_state;
}

// Separate polling functions for each encoder
void enc1_poll(void) {
    bool current_state = gpio_get(ENC1_DIG);
    enc_poll_common(&E1, current_state);
}

void enc2_poll(void) {
    bool current_state = gpio_get(ENC2_DIG);
    enc_poll_common(&E2, current_state);
}

// Main polling function to be called regularly
void motor_encoder_poll(void) {
    enc1_poll();
    enc2_poll();
}

/* ---------------- PWM helpers (unchanged) ---------------- */
void setup_pwm(uint pin){
    // Configure a GPIO pin to act as PWM output
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint32_t wrap = 9999; // resolution (0–9999)
    float div = (float)SYS_CLK_HZ / (F_PWM_HZ * (wrap + 1));
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(pin, 0); // start off
    pwm_set_enabled(slice, true); // enable PWM
}

// Set PWM duty cycle (0–100%)
void set_pwm_pct(uint pin, float pct){
    if (pct < 0) pct = 0; 
    if (pct > 100) pct = 100;
    uint slice = pwm_gpio_to_slice_num(pin);
    uint32_t top = pwm_hw->slice[slice].top;
    pwm_set_gpio_level(pin, (uint16_t)((pct/100.0f)*(top+1)));
}

/* ---------------- Motor drive (signed %) ---------------- */
void drive_signed(float left_pct, float right_pct){
    // left (M1)
    if (left_pct >= 0) { set_pwm_pct(M1A, left_pct); set_pwm_pct(M1B, 0); }
    else               { set_pwm_pct(M1A, 0);        set_pwm_pct(M1B, -left_pct); }
    // right (M2)
    if (right_pct >= 0){ set_pwm_pct(M2A, right_pct); set_pwm_pct(M2B, 0); }
    else               { set_pwm_pct(M2A, 0);         set_pwm_pct(M2B, -right_pct); }
}

// Stop both motors
void all_stop(void){ drive_signed(0,0); }

/* Reset encoder counters before each movement */
void enc_reset_move(void){
    E1.period_sum = E1.high_sum = E1.edges = E1.highs = 0;
    E2.period_sum = E2.high_sum = E2.edges = E2.highs = 0;
    // Also reset last_state to current state
    E1.last_state = gpio_get(ENC1_DIG);
    E2.last_state = gpio_get(ENC2_DIG);
}

/* -------------- One-shot move -------------- */
void run_for(float left_pct, float right_pct, uint32_t duration_ms){
    enc_reset_move();  // Clear previous encoder data
    drive_signed(left_pct, right_pct);  // Start moving

    // Keep motors running for the requested duration (ms)
    absolute_time_t t0 = get_absolute_time();
    while (absolute_time_diff_us(t0, get_absolute_time()) < (int64_t)duration_ms * 1000) {
        motor_encoder_poll();  // Poll encoders during movement
        tight_loop_contents();
    }

    all_stop(); // stop motors after time is up

    // compute averages (if we saw any pulses)
    float avg_per1 = (E1.edges ? (float)E1.period_sum / (float)E1.edges : 0.0f);
    float avg_hi1  = (E1.highs ? (float)E1.high_sum   / (float)E1.highs : 0.0f);
    float avg_per2 = (E2.edges ? (float)E2.period_sum / (float)E2.edges : 0.0f);
    float avg_hi2  = (E2.highs ? (float)E2.high_sum   / (float)E2.highs : 0.0f);

    // optional RPM (wheel) if period available
    float rpm1=0, rpm2=0;
    if (avg_per1 > 0.0f) {
        float freq1 = 1e6f / avg_per1;
        float rps_motor1 = freq1 / TICKS_PER_REV;
        rpm1 = (rps_motor1 / GEAR_RATIO) * 60.0f;
    }
    if (avg_per2 > 0.0f) {
        float freq2 = 1e6f / avg_per2;
        float rps_motor2 = freq2 / TICKS_PER_REV;
        rpm2 = (rps_motor2 / GEAR_RATIO) * 60.0f;
    }

    // Print one line of results after movement
    printf("DONE %ldms | L=%.0f%% R=%.0f%% | ENC1: pulses=%lu avg_high=%.0f us avg_period=%.0f us rpm=%.1f | "
           "ENC2: pulses=%lu avg_high=%.0f us avg_period=%.0f us rpm=%.1f\n",
           (long)duration_ms, left_pct, right_pct,
           (unsigned long)E1.edges, avg_hi1, avg_per1, rpm1,
           (unsigned long)E2.edges, avg_hi2, avg_per2, rpm2);
}

/* ---------------- Simplified motor encoder init (NO INTERRUPTS) ---------------- */
void motor_encoder_init(void) {
    // Initialize all PWM outputs for motors
    setup_pwm(M1A); setup_pwm(M1B); setup_pwm(M2A); setup_pwm(M2B);

    // Setup encoder inputs - NO INTERRUPTS
    gpio_init(ENC1_DIG); 
    gpio_set_dir(ENC1_DIG, GPIO_IN); 
    gpio_pull_up(ENC1_DIG);
    
    gpio_init(ENC2_DIG); 
    gpio_set_dir(ENC2_DIG, GPIO_IN); 
    gpio_pull_up(ENC2_DIG);

    // Initialize last_state with current state
    E1.last_state = gpio_get(ENC1_DIG);
    E2.last_state = gpio_get(ENC2_DIG);

    // Optional ADC
    adc_init(); adc_gpio_init(IR_ADC_PIN);

    printf("Motor system initialized (POLLING MODE - no interrupts)\n");
    printf("Pins: M1(8,9) M2(10,11) | ENC1 GP6, ENC2 GP4 | ADC0 GP26 (optional)\n");
}

/* ---------------- Serial command handling (unchanged) ---------------- */
void print_motor_help(void){
    printf(
      "\nOne-shot commands (run once, then stop):\n"
      "  help                         : this help\n"
      "  s                            : stop now\n"
      "  f <duty%> [ms]               : forward both wheels\n"
      "  b <duty%> [ms]               : backward both wheels\n"
      "  l <duty%> [ms]               : spin left (L back, R fwd)\n"
      "  r <duty%> [ms]               : spin right (L fwd, R back)\n"
      "  m <L%> <R%> [ms]             : set signed left/right (−100..100)\n"
      "Defaults: duty 30%%, duration 800 ms if omitted.\n\n"
    );
}

void process_motor_command(char* line) {
    char cmd[8]; int a=0, b=0, ms=800; // defaults
    int n = sscanf(line, "%7s %d %d %d", cmd, &a, &b, &ms);
    if (n >= 1) {
        if (!strcmp(cmd,"help")) print_motor_help();
        else if (!strcmp(cmd,"s")) { all_stop(); printf("STOP\n"); }
        else if (!strcmp(cmd,"r")) { 
            if(n<2) a=30; 
            if(n>=3) { ms=b; } 
            run_for(a,a, (n>=3?ms:800)); 
        }
        else if (!strcmp(cmd,"l")) { 
            if(n<2) a=30; 
            if(n>=3) { ms=b; } 
            run_for(-a,-a,(n>=3?ms:800)); 
        }
        else if (!strcmp(cmd,"f")) { 
            if(n<2) a=30; 
            if(n>=3) { ms=b; } 
            run_for(-a, a,(n>=3?ms:800)); 
        }
        else if (!strcmp(cmd,"b")) { 
            if(n<2) a=30; 
            if(n>=3) { ms=b; } 
            run_for( a,-a,(n>=3?ms:800)); 
        }
        else if (!strcmp(cmd,"m")) {
            if (n < 3) { printf("ERR: m needs L R [ms]\n"); }
            else {
                int L=a, R=b; 
                if(n>=4) ms=ms; else ms=800;
                if (L<-100) L=-100; 
                if(L>100) L=100;
                if (R<-100) R=-100; 
                if(R>100) R=100;
                run_for((float)L,(float)R,(uint32_t)ms);
            }
        } else {
            printf("ERR: unknown. Type 'help'.\n");
        }
    }
}