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

/* ---------------- PWM config ---------------- */
#define F_PWM_HZ     20000.0f
#define SYS_CLK_HZ 125000000u

/* ========== Encoder accumulators ========== */
typedef struct {
    volatile uint32_t pulse_count;
} enc_acc_t;

// One struct for each encoder
enc_acc_t E1 = {0}, E2 = {0};

/* ---------------- PWM helpers ---------------- */
void setup_pwm(uint pin){
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    uint32_t wrap = 9999;
    float div = (float)SYS_CLK_HZ / (F_PWM_HZ * (wrap + 1));
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, wrap);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

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

void all_stop(void){ drive_signed(0,0); }

void run_for(float left_pct, float right_pct, uint32_t duration_ms){
    drive_signed(left_pct, right_pct);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    all_stop();
}

/* ---------------- Motor encoder init (NO POLLING) ---------------- */
void motor_encoder_init(void) {
    // Initialize all PWM outputs for motors
    setup_pwm(M1A); setup_pwm(M1B); setup_pwm(M2A); setup_pwm(M2B);

    // Setup encoder inputs - timers will handle reading
    gpio_init(ENC1_DIG); 
    gpio_set_dir(ENC1_DIG, GPIO_IN); 
    gpio_pull_up(ENC1_DIG);
    
    gpio_init(ENC2_DIG); 
    gpio_set_dir(ENC2_DIG, GPIO_IN); 
    gpio_pull_up(ENC2_DIG);

    printf("Motor system initialized (Timer-based encoder reading)\n");
}

/* ---------------- Serial command handling ---------------- */
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
    char cmd[8]; int a=0, b=0, ms=800;
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