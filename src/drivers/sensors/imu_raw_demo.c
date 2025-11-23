#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "log.h"
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
    .offset_y = -132.0f, 
    .offset_z = -200.0f,
    .scale_x = 341.2f, 
    .scale_y = 341.2f, 
    .scale_z = 341.2f
};

// ========== Filter parameters ==========
#define ACCEL_ALPHA  0.05f
#define JERK_THRESHOLD   120.0f

// ========== OPTIMIZED Magnetometer Filter Parameters ==========
#define MAG_FILTER_SIZE 3  // REDUCED to 3 for minimal lag
#define BASELINE_MAG_STRENGTH 1.36f
#define STRENGTH_TOLERANCE 0.3f

// ========== Motion Detection (TUNED) ==========
#define ANGULAR_VELOCITY_THRESHOLD 5.0f   // Increased - faster rotation detection
#define STATIONARY_THRESHOLD 0.5f         // Tighter stationary detection

// ========== Globals ==========
static int filter_index = 0;
static int16_t ax_history[FILTER_SIZE] = {0};
static int16_t ay_history[FILTER_SIZE] = {0};
static int16_t az_history[FILTER_SIZE] = {0};
static float last_total_accel = 0;
static float pitch_filtered = 0.0f;
static float roll_filtered  = 0.0f;

// Magnetometer filtering - MINIMAL SIZE
static float mx_history[MAG_FILTER_SIZE] = {0};
static float my_history[MAG_FILTER_SIZE] = {0};
static int mag_filter_idx = 0;
static int mag_samples_count = 0;

// Heading tracking
static float heading_filtered = 0.0f;
static float heading_reference = 0.0f;
static bool heading_initialized = false;
static float last_valid_heading = 0.0f;
static float last_raw_heading = 0.0f;

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
    // stdio_init_all();
    sleep_ms(100);

    i2c_init(I2C_INST, 400000);  // INCREASED to 400kHz for faster reads
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    LOG_INFO("\n=== LSM303DLHC IMU Initialization ===\n");
    
    uint8_t acc_config[] = {0x20, 0x27};
    i2c_write_blocking(I2C_INST, ACC_ADDR, acc_config, 2, false);

    uint8_t mag_config[] = {0x02, 0x00};
    i2c_write_blocking(I2C_INST, MAG_ADDR, mag_config, 2, false);

    LOG_INFO("IMU Ready (400kHz I2C)\n\n");
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

// SIMPLE MEDIAN for small arrays
float median_3(float a, float b, float c) {
    if (a > b) {
        if (b > c) return b;      // a > b > c
        else if (a > c) return c; // a > c > b
        else return a;            // c > a > b
    } else {
        if (a > c) return a;      // b > a > c
        else if (b > c) return c; // b > c > a
        else return b;            // c > b > a
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

// ========== ULTRA-FAST HEADING with Aggressive Motion Detection ==========
float get_heading_fast(float *direction_str) {
    int16_t mx_raw, my_raw, mz_raw;
    if (!read_mag_raw(&mx_raw, &my_raw, &mz_raw)) {
        return heading_filtered;
    }

    // Apply calibration
    float mx = (mx_raw - mag_cal.offset_x) / mag_cal.scale_x;
    float my = (my_raw - mag_cal.offset_y) / mag_cal.scale_y;

    // === MINIMAL 3-SAMPLE MEDIAN FILTER ===
    mx_history[mag_filter_idx] = mx;
    my_history[mag_filter_idx] = my;
    mag_filter_idx = (mag_filter_idx + 1) % MAG_FILTER_SIZE;
    mag_samples_count = (mag_samples_count < MAG_FILTER_SIZE) ? mag_samples_count + 1 : MAG_FILTER_SIZE;
    
    float mx_median, my_median;
    
    if (mag_samples_count < 3) {
        // During warmup, use raw values
        mx_median = mx;
        my_median = my;
    } else {
        // Fast 3-value median
        mx_median = median_3(mx_history[0], mx_history[1], mx_history[2]);
        my_median = median_3(my_history[0], my_history[1], my_history[2]);
    }

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
        LOG_INFO("\n[INIT] Heading 0 deg = Current orientation (Baseline: %.2f)\n", mag_strength);
        return 0.0f;
    }

    // Convert to relative heading
    float relative_heading = angle_difference(heading_raw, heading_reference);
    relative_heading = normalize_angle(relative_heading);

    // === AGGRESSIVE MOTION-ADAPTIVE FILTERING ===
    float angular_velocity = fabsf(angle_difference(heading_raw, last_raw_heading));
    last_raw_heading = heading_raw;
    
    float alpha;

    if (angular_velocity > ANGULAR_VELOCITY_THRESHOLD) {
        alpha = has_interference ? 0.4f : 0.9f;
        LOG_INFO("[FAST] ");
        
    } else if (angular_velocity > STATIONARY_THRESHOLD) {
        alpha = has_interference ? 0.1f : 0.3f;
        LOG_INFO("[MED] ");
        
    } else {
        alpha = has_interference ? 0.01f : 0.02f;
        // Don't print anything when stationary
    }

    // Only show interference warning occasionally
    if (has_interference) {
        LOG_INFO("[UNSTABLE] Mag=%.2f", mag_strength);  // Just a warning symbol
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
    mag_samples_count = 0;
    LOG_INFO("\n[RESET] Heading reference reset\n");
}

// ========== Main Processing ==========
void compute_and_print_data(int16_t ax, int16_t ay, int16_t az) {
    int16_t ax_filtered = apply_filter(ax, ax_history, &filter_index);
    int16_t ay_filtered = apply_filter(ay, ay_history, &filter_index);
    int16_t az_filtered = apply_filter(az, az_history, &filter_index);

    float pitch, roll;
    calculate_orientation(ax_filtered, ay_filtered, az_filtered, &pitch, &roll);

    float direction = 0.0f;
    float heading = get_heading_fast(&direction);

    float ax_f = ax_filtered, ay_f = ay_filtered, az_f = az_filtered;
    remove_gravity_and_transform_frame(&ax_f, &ay_f, &az_f, pitch, roll);

    float total_accel = sqrtf(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
    float accel_change = fabsf(total_accel - last_total_accel);
    last_total_accel = total_accel;

    bool is_jerky = (accel_change > JERK_THRESHOLD);

    LOG_INFO("\nAcc X:%6d Y:%6d Z:%6d | ", ax_filtered, ay_filtered, az_filtered);
    LOG_INFO("Heading:%6.1f deg Pitch:%5.1f deg Roll:%5.1f deg | ", heading, pitch, roll);
    LOG_INFO("Jerk:%5.0f ", accel_change);
    
    if (direction > 0.5f) {
        LOG_INFO("CW ");
    } else if (direction < -0.5f) {
        LOG_INFO("CCW ");
    } else {
        LOG_INFO("---  ");
    }

    if (is_jerky) {
        LOG_INFO("[JERKY]");
    }
    
    LOG_INFO("\r");
    fflush(stdout);
}

// ========== Calibration ==========
void simple_calibration() {
    LOG_INFO("\n=== MAGNETOMETER CALIBRATION ===\n");
    LOG_INFO("Slowly rotate IMU in all directions for 30 seconds\n");
    LOG_INFO("Cover all orientations with figure-8 motions\n");
    LOG_INFO("IMPORTANT: Do this in the same location where you'll use the IMU!\n\n");

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
                LOG_INFO("X[%5d,%5d] Y[%5d,%5d] Z[%5d,%5d] N=%d\r",
                       min_x, max_x, min_y, max_y, min_z, max_z, samples);
            }
        }
        sleep_ms(50);
    }

    LOG_INFO("\n\n=== CALIBRATION RESULTS ===\n");
    
    mag_cal.offset_x = (max_x + min_x) / 2.0f;
    mag_cal.offset_y = (max_y + min_y) / 2.0f;
    mag_cal.offset_z = (max_z + min_z) / 2.0f;

    mag_cal.scale_x = (max_x - min_x) / 2.0f;
    mag_cal.scale_y = (max_y - min_y) / 2.0f;
    mag_cal.scale_z = (max_z - min_z) / 2.0f;

    LOG_INFO("Hard Iron Offsets: X=%.1f, Y=%.1f, Z=%.1f\n", 
        mag_cal.offset_x, mag_cal.offset_y, mag_cal.offset_z);
    LOG_INFO("Soft Iron Scales: X=%.1f, Y=%.1f, Z=%.1f\n", 
        mag_cal.scale_x, mag_cal.scale_y, mag_cal.scale_z);
    
    LOG_INFO("\n=== Update these values in your code: ===\n");
    LOG_INFO(".offset_x=%.1ff, .offset_y=%.1ff, .offset_z=%.1ff,\n",
           mag_cal.offset_x, mag_cal.offset_y, mag_cal.offset_z);
    LOG_INFO(".scale_x=%.1ff, .scale_y=%.1ff, .scale_z=%.1ff\n",
           mag_cal.scale_x, mag_cal.scale_y, mag_cal.scale_z);
}