#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_raw_demo.h"

#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E
#define I2C_INST i2c0
#define I2C_SDA_PIN 16 //GPIO 16 for SDA on Pico Robot Microcontroller, GPIO 20 for Personal Pico Microcontroller
#define I2C_SCL_PIN 17 //GPIO 17 for SCL on Pico Robot Microcontroller, GPIO 21 for Personal Pico Microcontroller
#define FILTER_SIZE 10
#define M_PI 3.14159265358979323846

// ========== Magnetometer calibration (replace after calibration) ==========
/*#define MAG_OFFSET_X  (-100.0f)
#define MAG_OFFSET_Y  (200.0f)
#define MAG_OFFSET_Z  (50.0f)
#define MAG_SCALE_X   (1.05f)
#define MAG_SCALE_Y   (0.97f)
#define MAG_SCALE_Z   (1.00f)*/

#define MAG_SCALE_X   (105.0f)
#define MAG_SCALE_Y   (38.5f)
#define MAG_SCALE_Z   (63.5f)
#define MAG_OFFSET_X  (-258.0f)
#define MAG_OFFSET_Y  (-248.5f)
#define MAG_OFFSET_Z  (-342.5f)

// ========== Filter parameters ==========
#define ACCEL_ALPHA  0.1f
#define YAW_ALPHA    0.2f

// ========== Globals ==========
static int filter_index = 0;
static int16_t ax_history[FILTER_SIZE] = {0};
static int16_t ay_history[FILTER_SIZE] = {0};
static int16_t az_history[FILTER_SIZE] = {0};
static float last_total_accel = 0;
static float pitch_filtered = 0.0f;
static float roll_filtered  = 0.0f;
static float yaw_filtered   = 0.0f;

// -------------------- INITIALIZATION --------------------
void imu_init() {
    stdio_init_all();
    sleep_ms(100);

    i2c_init(I2C_INST, 100);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("\nGY-511 LSM303DLHC Robot IMU Starting...\n");
    uint8_t acc_config[] = {0x20, 0x27};
    i2c_write_blocking(I2C_INST, ACC_ADDR, acc_config, 2, false);

    uint8_t mag_config[] = {0x02, 0x00};
    i2c_write_blocking(I2C_INST, MAG_ADDR, mag_config, 2, false);

    printf("Robot IMU System Active (Polling mode)\n\n");
}

// -------------------- SENSOR READINGS --------------------
bool read_accel_raw(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az) {
    uint8_t acc_data[6];
    uint8_t reg = 0x28 | 0x80;

    if (i2c_write_blocking(I2C_INST, ACC_ADDR, &reg, 1, true) != 1) return false;
    if (i2c_read_blocking(I2C_INST, ACC_ADDR, acc_data, 6, false) != 6) return false;

    *raw_ax = (int16_t)((acc_data[1] << 8) | acc_data[0]);
    *raw_ay = (int16_t)((acc_data[3] << 8) | acc_data[2]);
    *raw_az = (int16_t)((acc_data[5] << 8) | acc_data[4]);
    return true;
}

bool read_mag_raw(int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t mag_data[6];
    uint8_t reg = 0x03;
    if (i2c_write_blocking(I2C_INST, MAG_ADDR, &reg, 1, true) != 1) return false;
    if (i2c_read_blocking(I2C_INST, MAG_ADDR, mag_data, 6, false) != 6) return false;

    *mx = (int16_t)((mag_data[0] << 8) | mag_data[1]);
    *my = (int16_t)((mag_data[2] << 8) | mag_data[3]);
    *mz = (int16_t)((mag_data[4] << 8) | mag_data[5]); 
    return true;
}

// -------------------- FILTER --------------------
int16_t apply_filter(int16_t raw_value, int16_t history[], int *index) {
    history[*index] = raw_value;
    *index = (*index + 1) % FILTER_SIZE;
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += history[i];
    return (int16_t)(sum / FILTER_SIZE);
}

// -------------------- GRAVITY REMOVAL --------------------
void remove_gravity_and_transform_frame(float *ax, float *ay, float *az, float pitch, float roll) {
    float pitch_rad = pitch * M_PI / 180.0f;
    float roll_rad  = roll * M_PI / 180.0f;

    float gravity_x = 1000.0f * sinf(pitch_rad);
    float gravity_y = -1000.0f * sinf(roll_rad) * cosf(pitch_rad);
    float gravity_z = 1000.0f * cosf(roll_rad) * cosf(pitch_rad);

    float ax_no_grav = *ax - gravity_x;
    float ay_no_grav = *ay - gravity_y;
    float az_no_grav = *az - gravity_z;

    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);
    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);

    float x_rot = ax_no_grav;
    float y_rot = ay_no_grav * cos_r + az_no_grav * sin_r;
    float z_rot = -ay_no_grav * sin_r + az_no_grav * cos_r;

    *ax = x_rot * cos_p - z_rot * sin_p;
    *ay = y_rot;
    *az = x_rot * sin_p + z_rot * cos_p;
}

// -------------------- ORIENTATION --------------------
void calculate_orientation(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll) {
    float pitch_raw = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    float roll_raw  = atan2f(ay, az) * 180.0f / M_PI;

    pitch_filtered = ACCEL_ALPHA * pitch_raw + (1 - ACCEL_ALPHA) * pitch_filtered;
    roll_filtered  = ACCEL_ALPHA * roll_raw  + (1 - ACCEL_ALPHA) * roll_filtered;

    *pitch = pitch_filtered;
    *roll  = roll_filtered;
}

// -------------------- YAW CALCULATION --------------------
float calculate_yaw(int16_t mx, int16_t my, int16_t mz, float pitch_deg, float roll_deg) {
    // ===== Apply calibration (hard-iron and soft-iron correction) =====
    float mx_cal = (mx - MAG_OFFSET_X) / MAG_SCALE_X;
    float my_cal = (my - MAG_OFFSET_Y) / MAG_SCALE_Y;
    float mz_cal = (mz - MAG_OFFSET_Z) / MAG_SCALE_Z;

    // ===== Convert pitch/roll to radians =====
    float pitch = pitch_deg * M_PI / 180.0f;
    float roll  = roll_deg  * M_PI / 180.0f;

    // ===== Tilt compensation =====
    // Compensate the magnetic field measurements for tilt
    float Mx_comp = mx_cal * cosf(pitch) + mz_cal * sinf(pitch);
    float My_comp = mx_cal * sinf(roll) * sinf(pitch)
                  + my_cal * cosf(roll)
                  - mz_cal * sinf(roll) * cosf(pitch);

    // ===== Handle near-zero values to prevent NaNs =====
    if (fabsf(Mx_comp) < 1e-3f && fabsf(My_comp) < 1e-3f) {
        return yaw_filtered; // skip if unstable
    }

    // ===== Compute yaw in degrees =====
    float yaw_raw = atan2f(-My_comp, Mx_comp) * 180.0f / M_PI;

    // ===== Convert to navigation heading (0° = North, clockwise increase) =====
    yaw_raw = 90.0f - yaw_raw;
    if (yaw_raw < 0) yaw_raw += 360.0f;
    if (yaw_raw >= 360.0f) yaw_raw -= 360.0f;

    // ===== Low-pass filter for stability =====
    yaw_filtered = YAW_ALPHA * yaw_raw + (1 - YAW_ALPHA) * yaw_filtered;
    return yaw_filtered;
}

float calculate_simple_yaw(int16_t mx, int16_t my) {
    // Apply calibration to X and Y axes only
    float mx_cal = (mx - MAG_OFFSET_X) / MAG_SCALE_X;
    float my_cal = (my - MAG_OFFSET_Y) / MAG_SCALE_Y;
    
    // Basic yaw calculation (mathematical standard)
    float yaw = atan2f(-my_cal, mx_cal) * 180.0f / M_PI;
    
    // Convert to navigation standard (0° = North, clockwise)
    // yaw = 90.0f - yaw;
    if (yaw < 0) yaw += 360.0f;
    // if (yaw >= 360.0f) yaw -= 360.0f;
    
    // Add filtering
    yaw_filtered = YAW_ALPHA * yaw + (1 - YAW_ALPHA) * yaw_filtered;
    return yaw_filtered;
}

// -------------------- ANGLE NORMALIZATION --------------------
/*float normalize_angle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}*/

// -------------------- CALIBRATION TOOLS --------------------

// 1. Print raw magnetometer values for calibration
void print_raw_mag_calibration() {
    int16_t mx, my, mz;
    if (read_mag_raw(&mx, &my, &mz)) {
        printf("RAW_MAG: X:%6d, Y:%6d, Z:%6d\n", mx, my, mz);
    }
}

// 2. Simple calibration by finding min/max values
void simple_calibration() {
    printf("\n=== Starting Magnetometer Calibration ===\n");
    printf("Rotate the IMU in all directions for 30 seconds...\n");
    
    int16_t min_x = 32767, max_x = -32768;
    int16_t min_y = 32767, max_y = -32768;
    int16_t min_z = 32767, max_z = -32768;
    
    int successful_reads = 0;
    int failed_reads = 0;
    
    absolute_time_t start_time = get_absolute_time();
    
    while (absolute_time_diff_us(start_time, get_absolute_time()) < 30000000) {
        int16_t mx, my, mz;
        if (read_mag_raw(&mx, &my, &mz)) {
            successful_reads++;
            
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;
            
            printf("Success: %d | Failed: %d | X:%6d Y:%6d Z:%6d\r", 
                   successful_reads, failed_reads, mx, my, mz);
        } else {
            failed_reads++;
            printf("READ FAILED! Total fails: %d\r", failed_reads);
        }
        sleep_ms(100);
    }
    
    printf("\n\n=== CALIBRATION RESULTS ===\n");
    printf("Successful reads: %d, Failed reads: %d\n", successful_reads, failed_reads);
    
    if (successful_reads == 0) {
        printf("ERROR: No magnetometer readings! Check wiring.\n");
        return;
    }
    while(1){
        printf("Min: X:%6d Y:%6d Z:%6d\n", min_x, min_y, min_z);
        printf("Max: X:%6d Y:%6d Z:%6d\n", max_x, max_y, max_z);
        
        printf("\nHard Iron Offsets:\n");
        printf("#define MAG_OFFSET_X  (%.1ff)\n", (max_x + min_x) / 2.0f);
        printf("#define MAG_OFFSET_Y  (%.1ff)\n", (max_y + min_y) / 2.0f);
        printf("#define MAG_OFFSET_Z  (%.1ff)\n", (max_z + min_z) / 2.0f);
        
        printf("\nScale Factors:\n");
        printf("#define MAG_SCALE_X   (%.1ff)\n", (max_x - min_x) / 2.0f);
        printf("#define MAG_SCALE_Y   (%.1ff)\n", (max_y - min_y) / 2.0f);
        printf("#define MAG_SCALE_Z   (%.1ff)\n", (max_z - min_z) / 2.0f);
    }
    
}

// Add accelerometer calibration
void calibrate_accelerometer() {
    printf("\n=== Accelerometer Calibration ===\n");
    printf("Place sensor flat on table for +1g Z-axis...\n");
    sleep_ms(3000);
    
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int samples = 100;
    
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az;
        if (read_accel_raw(&ax, &ay, &az)) {
            sum_ax += ax;
            sum_ay += ay; 
            sum_az += az;
        }
        sleep_ms(10);
    }
    
    float offset_x = sum_ax / (float)samples;
    float offset_y = sum_ay / (float)samples;
    float offset_z = (sum_az / (float)samples) - 1000.0f; // -1g for Z when flat
    
    printf("Accelerometer Offsets:\n");
    printf("X: %.1f, Y: %.1f, Z: %.1f\n", offset_x, offset_y, offset_z);
}

// -------------------- COMPUTE AND PRINT --------------------
void compute_and_print_data(int16_t ax, int16_t ay, int16_t az) {
    int16_t ax_filtered = apply_filter(ax, ax_history, &filter_index);
    int16_t ay_filtered = apply_filter(ay, ay_history, &filter_index);
    int16_t az_filtered = apply_filter(az, az_history, &filter_index);

    int16_t mx, my, mz;
    if (!read_mag_raw(&mx, &my, &mz)) {
        printf(" MAG_FAILED\n");
        return;
    }

    float pitch, roll;
    calculate_orientation(ax_filtered, ay_filtered, az_filtered, &pitch, &roll);

    // ✅ Use the tilt-compensated yaw function
    float yaw = calculate_simple_yaw(mx, my);

    // Transform accelerometer readings and detect motion
    float ax_f = ax_filtered, ay_f = ay_filtered, az_f = az_filtered;
    remove_gravity_and_transform_frame(&ax_f, &ay_f, &az_f, pitch, roll);

    float total_accel = sqrtf(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
    float accel_change = fabsf(total_accel - last_total_accel);
    last_total_accel = total_accel;
    bool is_jerky = (accel_change > 200);

    printf("\nX:%6d Y:%6d Z:%6d | ", ax_filtered, ay_filtered, az_filtered);
    printf("Yaw:%6.1f° Pitch:%6.1f° Roll:%6.1f° | Jerk:%5.0f", yaw, pitch, roll, accel_change);
    if (is_jerky) printf(" [JERKY]");
    else printf("         ");
}


// -------------------- MAIN LOOP (TEST DIFFERENT MODES) --------------------
// int main(void) {
//     stdio_init_all();
//     imu_init();

//     // Wait for serial to be ready
//     sleep_ms(5000);
    
//     printf("\n\n=== IMU CALIBRATION TOOL ===\n");
//     printf("Choose mode:\n");
//     printf("1 - Normal operation\n");
//     printf("2 - Raw magnetometer values\n");
//     printf("3 - Run calibration\n");
//     printf("Enter choice (1, 2, or 3): ");
    
//     // Clear any existing serial buffer
//     while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);
    
//     // Wait for user input with timeout
//     absolute_time_t timeout = make_timeout_time_ms(10000); // 10 second timeout
//     char choice = '3';
    
//     // while (absolute_time_diff_us(get_absolute_time(), timeout) > 0) {
//     //     int ch = getchar_timeout_us(100000); // 100ms timeout
//     //     if (ch != PICO_ERROR_TIMEOUT) {
//     //         choice = (char)ch;
//     //         break;
//     //     }
//     // }
    
//     if (choice == 0) {
//         printf("\nNo input - starting normal mode\n");
//         choice = '1';
//     }
    
//     printf("\nStarting mode: %c\n\n", choice);
    
//     if (choice == '2') {
//         printf("=== RAW MAGNETOMETER MODE ===\n");
//         printf("Press any key to exit...\n");
//         while (true) {
//             print_raw_mag_calibration();
//             sleep_ms(500);
//             // Check for exit key
//             if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) break;
//         }
//     }
//     else if (choice == '3') {
//         simple_calibration();
//         printf("\nCalibration complete! Copy the values above into your code.\n");
//         printf("Press any key to exit...\n");
//         //while (getchar_timeout_us(1000000) == PICO_ERROR_TIMEOUT);
//     }
//     else {
//         printf("=== NORMAL OPERATION MODE ===\n");
//         printf("Press any key to change mode...\n");
//         //simple_calibration();
//         while (true) {
//             int16_t ax, ay, az;
//             if (read_accel_raw(&ax, &ay, &az)) {
//                 compute_and_print_data(ax, ay, az);
//             } else {
//                 printf("Accelerometer read failed!\n");
//             }
            
//             // Check for mode change
//             // if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
//             //     printf("\n\nRestart to change mode.\n");
//             //     break;
//             // }
            
//             sleep_ms(100);
//         }
//     }
    
//     return 0;
// }