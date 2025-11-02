#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu_raw_demo.h"

#define ACC_ADDR 0x19
#define MAG_ADDR 0x1E
#define I2C_INST i2c0
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17
#define FILTER_SIZE 10
#define M_PI 3.14159265358979323846

// ========== Magnetometer Calibration ==========
typedef struct {
    float offset_x, offset_y, offset_z;
    float scale_x, scale_y, scale_z;
} mag_calibration_t;

static mag_calibration_t mag_cal = {
    .offset_x = 10.0f, 
    .offset_y = -126.0f, 
    .offset_z = -200.0f,
    .scale_x = 341.2f, 
    .scale_y = 341.2f, 
    .scale_z = 341.2f
};

// ========== Filter parameters ==========
#define ACCEL_ALPHA  0.05f
#define JERK_THRESHOLD   120.0f

// ========== Magnetometer Filter Parameters ==========
#define MAG_FILTER_SIZE 5  // REDUCED from 10 for faster response
#define BASELINE_MAG_STRENGTH 1.36f
#define STRENGTH_TOLERANCE 0.3f

// ========== NEW: Motion Detection Parameters ==========
#define ANGULAR_VELOCITY_THRESHOLD 3.0f  // degrees per sample to detect rotation
#define STATIONARY_THRESHOLD 1.0f        // degrees per sample for stationary

// ========== Globals ==========
static int filter_index = 0;
static int16_t ax_history[FILTER_SIZE] = {0};
static int16_t ay_history[FILTER_SIZE] = {0};
static int16_t az_history[FILTER_SIZE] = {0};
static float last_total_accel = 0;
static float pitch_filtered = 0.0f;
static float roll_filtered  = 0.0f;

// Magnetometer filtering - REDUCED SIZE
static float mx_history[MAG_FILTER_SIZE] = {0};
static float my_history[MAG_FILTER_SIZE] = {0};
static int mag_filter_idx = 0;
static int mag_samples_count = 0;  // Track warmup

// Heading tracking
static float heading_filtered = 0.0f;
static float heading_reference = 0.0f;
static bool heading_initialized = false;
static float last_valid_heading = 0.0f;
static float last_raw_heading = 0.0f;  // NEW: For velocity calculation

// ========== Angle utilities ==========
float normalize_angle(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

float angle_difference(float a, float b) {
    float diff = a - b;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

// ========== I2C Initialization ==========
void imu_init() {
    //stdio_init_all();
    sleep_ms(100);

    i2c_init(I2C_INST, 100000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("\n=== LSM303DLHC IMU Initialization ===\n");
    
    uint8_t acc_config[] = {0x20, 0x27};
    i2c_write_blocking(I2C_INST, ACC_ADDR, acc_config, 2, false);

    uint8_t mag_config[] = {0x02, 0x00};
    i2c_write_blocking(I2C_INST, MAG_ADDR, mag_config, 2, false);

    printf("IMU Ready\n\n");
}

// ========== Sensor Reading ==========
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
    *mz = (int16_t)((mag_data[2] << 8) | mag_data[3]);
    *my = (int16_t)((mag_data[4] << 8) | mag_data[5]);
    return true;
}

// ========== Filters ==========
int16_t apply_filter(int16_t raw_value, int16_t history[], int *index) {
    history[*index] = raw_value;
    *index = (*index + 1) % FILTER_SIZE;
    int32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) sum += history[i];
    return (int16_t)(sum / FILTER_SIZE);
}

void sort_float_array(float *arr, int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

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

void calculate_orientation(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll) {
    float pitch_raw = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    float roll_raw  = atan2f(ay, az) * 180.0f / M_PI;

    pitch_filtered = ACCEL_ALPHA * pitch_raw + (1.0f - ACCEL_ALPHA) * pitch_filtered;
    roll_filtered  = ACCEL_ALPHA * roll_raw  + (1.0f - ACCEL_ALPHA) * roll_filtered;

    *pitch = pitch_filtered;
    *roll  = roll_filtered;
}

// ========== FAST RESPONSE HEADING with Motion-Adaptive Filtering ==========
float get_heading_fast(float *direction_str) {
    int16_t mx_raw, my_raw, mz_raw;
    if (!read_mag_raw(&mx_raw, &my_raw, &mz_raw)) {
        return heading_filtered;
    }

    // Apply calibration
    float mx = (mx_raw - mag_cal.offset_x) / mag_cal.scale_x;
    float my = (my_raw - mag_cal.offset_y) / mag_cal.scale_y;

    // === LIGHTWEIGHT MEDIAN FILTER (smaller window) ===
    mx_history[mag_filter_idx] = mx;
    my_history[mag_filter_idx] = my;
    mag_filter_idx = (mag_filter_idx + 1) % MAG_FILTER_SIZE;
    mag_samples_count = (mag_samples_count < MAG_FILTER_SIZE) ? mag_samples_count + 1 : MAG_FILTER_SIZE;
    
    // Only use available samples during warmup
    int samples_to_use = mag_samples_count;
    
    float mx_sorted[MAG_FILTER_SIZE];
    memcpy(mx_sorted, mx_history, samples_to_use * sizeof(float));
    sort_float_array(mx_sorted, samples_to_use);
    float mx_median = mx_sorted[samples_to_use / 2];
    
    float my_sorted[MAG_FILTER_SIZE];
    memcpy(my_sorted, my_history, samples_to_use * sizeof(float));
    sort_float_array(my_sorted, samples_to_use);
    float my_median = my_sorted[samples_to_use / 2];

    // === MAGNETIC FIELD STRENGTH CHECK ===
    float mag_strength = sqrtf(mx_median * mx_median + my_median * my_median);
    bool has_interference = (fabsf(mag_strength - BASELINE_MAG_STRENGTH) > STRENGTH_TOLERANCE);
    
    // Calculate heading from filtered magnetometer data
    float heading_raw = atan2f(my_median, mx_median) * 180.0f / M_PI;
    heading_raw = normalize_angle(heading_raw);

    // Initialize reference on first valid reading
    if (!heading_initialized) {
        heading_reference = heading_raw;
        heading_filtered = 0.0f;
        heading_initialized = true;
        last_valid_heading = 0.0f;
        last_raw_heading = heading_raw;
        printf("\n[INIT] Heading 0° = Current orientation (Baseline: %.2f)\n", mag_strength);
        return 0.0f;
    }

    // Convert to relative heading
    float relative_heading = angle_difference(heading_raw, heading_reference);
    relative_heading = normalize_angle(relative_heading);

    // === MOTION-ADAPTIVE FILTERING (Key optimization!) ===
    float angular_velocity = fabsf(angle_difference(heading_raw, last_raw_heading));
    last_raw_heading = heading_raw;
    
    float alpha;
    
    if (has_interference) {
        // During interference, still responsive but more cautious
        if (angular_velocity > ANGULAR_VELOCITY_THRESHOLD) {
            alpha = 0.4f;  // FAST even during interference if rotating
            printf("[INT+ROT:%.2f] ", mag_strength);
        } else {
            alpha = 0.1f;  // Slower when stationary with interference
            printf("[INT:%.2f] ", mag_strength);
        }
    } else {
        // Clean signal - use velocity-based adaptation
        if (angular_velocity > ANGULAR_VELOCITY_THRESHOLD) {
            alpha = 0.7f;  // VERY FAST during rotation
            printf("[FAST↻] ");
        } else if (angular_velocity > STATIONARY_THRESHOLD) {
            alpha = 0.4f;  // Medium speed for slow rotation
        } else {
            alpha = 0.2f;  // Slower when stationary for stability
        }
    }
    
    // Apply exponential filter with adaptive alpha
    float diff_to_filter = angle_difference(relative_heading, heading_filtered);
    heading_filtered = normalize_angle(heading_filtered + alpha * diff_to_filter);
    
    // Direction indicator (CW/CCW)
    if (direction_str) {
        float rotation_speed = angle_difference(heading_filtered, last_valid_heading);
        if (rotation_speed > 0.5f) {
            *direction_str = 1.0f;  // Clockwise
        } else if (rotation_speed < -0.5f) {
            *direction_str = -1.0f;  // Counter-clockwise
        } else {
            *direction_str = 0.0f;  // Stationary
        }
    }
    
    last_valid_heading = heading_filtered;
    return heading_filtered;
}

void reset_heading_reference() {
    heading_initialized = false;
    mag_samples_count = 0;  // Reset filter warmup
    printf("\n[RESET] Heading reference reset\n");
}

// ========== Main Processing ==========
void compute_and_print_data(int16_t ax, int16_t ay, int16_t az) {
    int16_t ax_filtered = apply_filter(ax, ax_history, &filter_index);
    int16_t ay_filtered = apply_filter(ay, ay_history, &filter_index);
    int16_t az_filtered = apply_filter(az, az_history, &filter_index);

    float pitch, roll;
    calculate_orientation(ax_filtered, ay_filtered, az_filtered, &pitch, &roll);

    // Use fast heading function
    float direction = 0.0f;
    float heading = get_heading_fast(&direction);

    float ax_f = ax_filtered, ay_f = ay_filtered, az_f = az_filtered;
    remove_gravity_and_transform_frame(&ax_f, &ay_f, &az_f, pitch, roll);

    float total_accel = sqrtf(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
    float accel_change = fabsf(total_accel - last_total_accel);
    last_total_accel = total_accel;

    bool is_jerky = (accel_change > JERK_THRESHOLD);

    printf("\nAcc X:%6d Y:%6d Z:%6d | ", ax_filtered, ay_filtered, az_filtered);
    printf("Heading:%6.1f° Pitch:%5.1f° Roll:%5.1f° | ", heading, pitch, roll);
    printf("Jerk:%5.0f ", accel_change);
    
    if (direction > 0.5f) {
        printf("CW↻  ");
    } else if (direction < -0.5f) {
        printf("CCW↺ ");
    } else {
        printf("---  ");
    }

    if (is_jerky) {
        printf("[JERKY]");
    }
    
    printf("\r");
    fflush(stdout);
}

// ========== Calibration ==========
void simple_calibration() {
    printf("\n=== MAGNETOMETER CALIBRATION ===\n");
    printf("Slowly rotate IMU in all directions for 30 seconds\n");
    printf("Cover all orientations with figure-8 motions\n");
    printf("IMPORTANT: Do this in the same location where you'll use the IMU!\n\n");

    int16_t min_x = 32767, max_x = -32768;
    int16_t min_y = 32767, max_y = -32768;
    int16_t min_z = 32767, max_z = -32768;

    absolute_time_t start = get_absolute_time();
    int samples = 0;

    while (absolute_time_diff_us(start, get_absolute_time()) < 30000000) {
        int16_t mx, my, mz;
        if (read_mag_raw(&mx, &my, &mz)) {
            samples++;
            if (mx < min_x) min_x = mx;
            if (mx > max_x) max_x = mx;
            if (my < min_y) min_y = my;
            if (my > max_y) max_y = my;
            if (mz < min_z) min_z = mz;
            if (mz > max_z) max_z = mz;

            if (samples % 10 == 0) {
                printf("X[%5d,%5d] Y[%5d,%5d] Z[%5d,%5d] N=%d\r",
                       min_x, max_x, min_y, max_y, min_z, max_z, samples);
            }
        }
        sleep_ms(50);
    }

    printf("\n\n=== CALIBRATION RESULTS ===\n");
    
    mag_cal.offset_x = (max_x + min_x) / 2.0f;
    mag_cal.offset_y = (max_y + min_y) / 2.0f;
    mag_cal.offset_z = (max_z + min_z) / 2.0f;

    mag_cal.scale_x = (max_x - min_x) / 2.0f;
    mag_cal.scale_y = (max_y - min_y) / 2.0f;
    mag_cal.scale_z = (max_z - min_z) / 2.0f;

    printf("Hard Iron Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", 
        mag_cal.offset_x, mag_cal.offset_y, mag_cal.offset_z);
    printf("Soft Iron Scales: X=%.1f, Y=%.1f, Z=%.1f\n", 
        mag_cal.scale_x, mag_cal.scale_y, mag_cal.scale_z);
    
    printf("\n=== Update these values in your code: ===\n");
    printf(".offset_x=%.1ff, .offset_y=%.1ff, .offset_z=%.1ff,\n",
           mag_cal.offset_x, mag_cal.offset_y, mag_cal.offset_z);
    printf(".scale_x=%.1ff, .scale_y=%.1ff, .scale_z=%.1ff\n",
           mag_cal.scale_x, mag_cal.scale_y, mag_cal.scale_z);
}

// ========== Main ==========
// int main(void) {
//     stdio_init_all();
//     imu_init();
//     sleep_ms(2000);

//     printf("\n=== IMU HEADING TRACKER (FAST RESPONSE) ===\n");
//     printf("Features: Motion-adaptive filtering for instant response\n");
//     printf("1 - Normal operation\n");
//     printf("2 - Calibrate magnetometer\n");
//     printf("Choice: ");

//     while(getchar_timeout_us(0) != PICO_ERROR_TIMEOUT);

//     absolute_time_t timeout = make_timeout_time_ms(5000);
//     char choice = '1';

//     while (absolute_time_diff_us(get_absolute_time(), timeout) > 0) {
//         int ch = getchar_timeout_us(100000);
//         if (ch != PICO_ERROR_TIMEOUT) {
//             choice = (char)ch;
//             break;
//         }
//     }

//     printf("%c\n\n", choice);

//     if (choice == '2') {
//         simple_calibration();
//         printf("\nPress any key to start...\n");
//         while (getchar_timeout_us(1000000) == PICO_ERROR_TIMEOUT);
//     }

//     printf("\n=== OPERATION ===\n");
//     printf("0° = Initial orientation (North)\n");
//     printf("Rotation: CW increases (0°→90°→180°→270°→360°)\n");
//     printf("Press 'R' to reset reference\n");
//     printf("Watch for [FAST↻] = Rapid rotation detected (instant response)\n");
//     printf("Watch for [INT] = Magnetic interference (still responsive)\n\n");

//     sleep_ms(1000);

//     while (true) {
//         int16_t ax, ay, az;
//         if (read_accel_raw(&ax, &ay, &az)) {
//             compute_and_print_data(ax, ay, az);
//         }

//         int ch = getchar_timeout_us(0);
//         if (ch == 'r' || ch == 'R') {
//             reset_heading_reference();
//         }

//         sleep_ms(100);
//     }

//     return 0;
// }