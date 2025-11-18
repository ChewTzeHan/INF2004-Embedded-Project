#include "ir_sensor.h"
#include "hardware/adc.h"
#include <stdbool.h>
#include "pico/time.h"
#include <stdio.h>  // Added missing include

void ir_init(ir_calib_t *cal) {
    adc_init();
    adc_gpio_init(IR_GPIO);
    adc_select_input(IR_ADC_INPUT);
    if (cal) {
        cal->min_raw = 4095;
        cal->max_raw = 0;
    }
    printf("IR: ADC initialized on GPIO%d\n", IR_GPIO);
}

void ir_digital_init(void) {
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN); // Optional: add pull-up if needed
}

uint16_t ir_read_raw(void) {
    (void)adc_read();
    sleep_us(5);
    const int N = 12;
    uint32_t acc = 0;
    for (int i = 0; i < N; i++) {
        acc += adc_read();
        sleep_us(5);
    }
    
    return (uint16_t)(acc / N);
}

bool ir_read_digital(void) {
    return gpio_get(RIGHT_IR_DIGITAL_PIN);
}

bool ir_is_black(uint16_t sample, const ir_calib_t *cal) {
    uint16_t th = ir_threshold(cal);
#if IR_BLACK_IS_LOWER
    return sample < th;
#else
    return sample > th;
#endif
}

int ir_classify(uint16_t sample, const ir_calib_t *cal) {
    return ir_is_black(sample, cal) ? 1 : 0;
}

int ir_classify_hysteresis(uint16_t sample, const ir_calib_t *cal, int prev_state) {
    uint16_t th = ir_threshold(cal);
    uint16_t low = (th > IR_HYST_MARGIN) ? (th - IR_HYST_MARGIN) : 0;
    uint16_t high = (th + IR_HYST_MARGIN <= 400) ? (th + IR_HYST_MARGIN) : 400;

#if IR_BLACK_IS_LOWER
    // BLACK when below 'low', WHITE when above 'high', otherwise keep previous state
    if (sample <= low) return 1;          // definitely black
    if (sample >= high) return 0;         // definitely white
    return prev_state;
#else
    // Inverted behaviour
    if (sample >= high) return 1;         // definitely black (higher = black)
    if (sample <= low) return 0;          // definitely white
    return prev_state;
#endif
}

int classify_colour(uint16_t raw){
    if (raw > 1000) return 1; // Black
    else if (raw > 400 && raw < 1000) return 2; // Grey
    else return 0; // White
}

// ===================== INTERRUPT TESTING =====================
// Uncomment the line below to enable standalone testing
// #define IR_SENSOR_STANDALONE_TEST

#ifdef IR_SENSOR_STANDALONE_TEST

// Global variables for interrupt testing
static volatile uint32_t g_irq_count = 0;
static volatile uint32_t g_last_irq_time = 0;
static volatile uint32_t g_irq_durations[100];
static volatile uint8_t g_irq_index = 0;
static volatile bool g_first_edge = true;

// Interrupt handler for digital IR sensor testing
static void digital_ir_test_isr(uint gpio, uint32_t events) {
    uint32_t now = time_us_32();
    
    if (g_first_edge) {
        // First edge - just record time
        g_last_irq_time = now;
        g_first_edge = false;
        printf("FIRST EDGE: events=0x%lx, state=%d\n", events, gpio_get(gpio));
    } else {
        // Calculate time since last edge
        uint32_t duration = now - g_last_irq_time;
        g_last_irq_time = now;
        
        if (g_irq_index < 100) {
            g_irq_durations[g_irq_index] = duration;
            g_irq_index++;
        }
        
        g_irq_count++;
        
        printf("IRQ #%lu: events=0x%lx, duration=%luus, state=%d\n", 
               g_irq_count, events, duration, gpio_get(gpio));
    }
}

// Test digital IR sensor with different pull configurations
void test_digital_ir_pull_configs(void) {
    printf("\n=== DIGITAL IR PULL CONFIGURATION TEST ===\n");
    
    const char* config_names[] = {"Pull-Up", "No Pull", "Pull-Down"};
    
    for (int config = 0; config < 3; config++) {
        printf("\n--- Testing %s ---\n", config_names[config]);
        
        // Initialize GPIO
        gpio_init(RIGHT_IR_DIGITAL_PIN);
        gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
        
        // Set pull configuration
        switch (config) {
            case 0: // Pull-Up
                gpio_pull_up(RIGHT_IR_DIGITAL_PIN);
                break;
            case 1: // No Pull
                gpio_disable_pulls(RIGHT_IR_DIGITAL_PIN);
                break;
            case 2: // Pull-Down
                gpio_pull_down(RIGHT_IR_DIGITAL_PIN);
                break;
        }
        
        // Read initial state
        printf("Initial state: %d\n", ir_read_digital());
        
        // Read multiple samples
        printf("Sampling 10 readings: ");
        for (int i = 0; i < 10; i++) {
            printf("%d ", ir_read_digital());
            sleep_ms(100);
        }
        printf("\n");
        
        sleep_ms(500);
    }
}

// Test interrupt functionality
void test_digital_ir_interrupts(void) {
    printf("\n=== DIGITAL IR INTERRUPT TEST ===\n");
    
    // Reset test variables
    g_irq_count = 0;
    g_last_irq_time = 0;
    g_irq_index = 0;
    g_first_edge = true;
    
    // Configure GPIO for interrupts
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);  // Start with pull-up
    
    printf("GPIO%d configured with pull-up\n", RIGHT_IR_DIGITAL_PIN);
    printf("Initial state: %d\n", ir_read_digital());
    
    // Clear any pending interrupts
    gpio_acknowledge_irq(RIGHT_IR_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
    
    // Enable interrupts for both edges
    gpio_set_irq_enabled_with_callback(RIGHT_IR_DIGITAL_PIN, 
                                      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                      true, &digital_ir_test_isr);
    
    printf("Interrupts enabled - waiting for edges...\n");
    printf("Manually trigger the sensor or short GPIO%d to GND/3V3\n", RIGHT_IR_DIGITAL_PIN);
    printf("Press any key to stop...\n\n");
    
    uint32_t start_time = time_us_32();
    
    while (true) {
        // Check for keyboard input to stop
        int c = getchar_timeout_us(100000); // 100ms timeout
        if (c != PICO_ERROR_TIMEOUT) {
            break;
        }
        
        // Print status every 2 seconds
        static uint32_t last_status = 0;
        uint32_t now = time_us_32();
        if (now - last_status > 2000000) {
            printf("Status: %lu IRQs in %lu ms (current state: %d)\n", 
                   g_irq_count, (now - start_time) / 1000, 
                   ir_read_digital());
            last_status = now;
        }
    }
    
    // Disable interrupts
    gpio_set_irq_enabled(RIGHT_IR_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    
    printf("\n=== INTERRUPT TEST RESULTS ===\n");
    printf("Total interrupts: %lu\n", g_irq_count);
    printf("Test duration: %lu ms\n", (time_us_32() - start_time) / 1000);
    
    if (g_irq_count > 0) {
        printf("First 10 durations: ");
        for (int i = 0; i < 10 && i < g_irq_index; i++) {
            printf("%luus ", g_irq_durations[i]);
        }
        printf("\n");
    }
}

// Manual barcode simulation test
void test_manual_barcode_simulation(void) {
    printf("\n=== MANUAL BARCODE SIMULATION TEST ===\n");
    printf("Slowly move a barcode under the sensor\n");
    printf("We'll capture transitions for 15 seconds\n\n");
    
    // Reset test variables
    g_irq_count = 0;
    g_last_irq_time = 0;
    g_irq_index = 0;
    g_first_edge = true;
    
    // Configure GPIO for interrupts
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);
    
    // Enable interrupts
    gpio_set_irq_enabled_with_callback(RIGHT_IR_DIGITAL_PIN, 
                                      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, 
                                      true, &digital_ir_test_isr);
    
    printf("Starting capture in 3 seconds...\n");
    sleep_ms(3000);
    
    printf("CAPTURING NOW! Move barcode slowly under sensor...\n");
    
    uint32_t start_time = time_us_32();
    uint32_t last_irq_count = 0;
    
    while (time_us_32() - start_time < 15000000) { // 15 seconds
        // Print progress every second
        static uint32_t last_print = 0;
        uint32_t now = time_us_32();
        
        if (now - last_print > 1000000) {
            uint32_t elapsed = (now - start_time) / 1000;
            uint32_t new_irqs = g_irq_count - last_irq_count;
            
            printf("Time: %lums, IRQs: %lu (+%lu this second)\n", 
                   elapsed, g_irq_count, new_irqs);
            
            last_irq_count = g_irq_count;
            last_print = now;
        }
        
        // Stop early if we get a lot of transitions (success case)
        if (g_irq_count > 50) {
            printf("Got 50+ transitions - stopping early\n");
            break;
        }
        
        sleep_ms(100);
    }
    
    // Disable interrupts
    gpio_set_irq_enabled(RIGHT_IR_DIGITAL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
    
    printf("\n=== BARCODE SIMULATION RESULTS ===\n");
    printf("Total transitions: %lu\n", g_irq_count);
    printf("Capture time: %lu ms\n", (time_us_32() - start_time) / 1000);
    
    if (g_irq_count > 0) {
        printf("Transition pattern (first 20): ");
        for (int i = 0; i < 20 && i < g_irq_index; i++) {
            // Classify as narrow (< 5000us) or wide (>= 5000us)
            if (g_irq_durations[i] < 5000) {
                printf("N");
            } else {
                printf("W");
            }
        }
        printf("\n");
        
        printf("Raw durations (us): ");
        for (int i = 0; i < 10 && i < g_irq_index; i++) {
            printf("%lu ", g_irq_durations[i]);
        }
        printf("\n");
    } else {
        printf("ERROR: No transitions detected!\n");
        printf("Check: Sensor wiring, barcode contrast, movement speed\n");
    }
}

// Simple polling test to verify basic sensor functionality
void test_basic_sensor_operation(void) {
    printf("\n=== BASIC SENSOR OPERATION TEST ===\n");
    
    gpio_init(RIGHT_IR_DIGITAL_PIN);
    gpio_set_dir(RIGHT_IR_DIGITAL_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_IR_DIGITAL_PIN);
    
    printf("Testing sensor for 10 seconds using polling...\n");
    printf("Move sensor between black and white surfaces\n\n");
    
    bool last_state = ir_read_digital();
    uint32_t transitions = 0;
    uint32_t start_time = time_us_32();
    
    printf("Starting state: %d\n", last_state);
    
    while (time_us_32() - start_time < 10000000) { // 10 seconds
        bool current_state = ir_read_digital();
        
        if (current_state != last_state) {
            transitions++;
            printf("Transition %lu: %d -> %d\n", transitions, last_state, current_state);
            last_state = current_state;
        }
        
        sleep_ms(10); // 10ms polling
    }
    
    printf("\n=== BASIC TEST RESULTS ===\n");
    printf("Total transitions detected: %lu\n", transitions);
    printf("Final sensor state: %d\n", ir_read_digital());
    
    if (transitions == 0) {
        printf("WARNING: No transitions detected in 10 seconds!\n");
        printf("Sensor may be stuck or not properly connected.\n");
    }
}

// Main function for standalone testing
int main(void) {
    stdio_init_all();
    
    // Wait for USB to initialize
    sleep_ms(2000);
    
    printf("\n=== DIGITAL IR SENSOR STANDALONE TEST ===\n");
    printf("Sensor GPIO: %d\n", RIGHT_IR_DIGITAL_PIN);
    
    while (true) {
        printf("\n=== TEST MENU ===\n");
        printf("1. Test Pull Configurations\n");
        printf("2. Test Basic Sensor Operation (Polling)\n");
        printf("3. Test Interrupts (Manual Trigger)\n");
        printf("4. Test Manual Barcode Scanning\n");
        printf("5. Exit\n");
        printf("Choose test (1-5): ");
        
        char choice = getchar();
        printf("\n");
        
        switch (choice) {
            case '1':
                test_digital_ir_pull_configs();
                break;
                
            case '2':
                test_basic_sensor_operation();
                break;
                
            case '3':
                test_digital_ir_interrupts();
                break;
                
            case '4':
                test_manual_barcode_simulation();
                break;
                
            case '5':
                printf("Exiting test...\n");
                return 0;
                
            default:
                printf("Invalid choice. Please enter 1-5.\n");
                break;
        }
        
        // Clear any remaining input
        while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
        
        printf("\nPress any key to continue...");
        getchar();
        sleep_ms(500);
    }
    
    return 0;
}

#endif // IR_SENSOR_STANDALONE_TEST