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
#define I2C_SDA_PIN 16
#define I2C_SCL_PIN 17
#define FILTER_SIZE 10

// Global variables
static int filter_index = 0;
static int16_t ax_history[FILTER_SIZE] = {0};
static int16_t ay_history[FILTER_SIZE] = {0};
static int16_t az_history[FILTER_SIZE] = {0};
static float last_total_accel = 0;

// -------------------- FUNCTION DECLARATIONS --------------------
void imu_init();
void imu_configure_sensors();
bool read_accel_raw(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az);
bool read_mag_raw(int16_t *mx, int16_t *my);
int16_t apply_filter(int16_t raw_value, int16_t history[], int *index);
void compute_and_print_data(int16_t ax, int16_t ay, int16_t az);
float calculate_heading(int16_t mx, int16_t my);
void remove_gravity_and_transform_frame(float *ax, float *ay, float *az, float pitch, float roll);
void calculate_orientation(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll);

// -------------------- INITIALIZATION --------------------
void imu_init() {
    stdio_init_all();
    sleep_ms(100);

    // Initialize I2C WITHOUT interrupts
    i2c_init(I2C_INST, 100);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    printf("\nGY-511 LSM303DLHC Robot IMU Starting...\n");
    imu_configure_sensors();

    printf("Robot IMU System Active (POLLING MODE)\n");
    printf("• Using polling I2C (no interrupts)\n");
    printf("• All sensors integrated\n\n");

    
}

void imu_configure_sensors() {
    uint8_t acc_config[] = {0x20, 0x27};  // enable accel
    i2c_write_blocking(I2C_INST, ACC_ADDR, acc_config, 2, false);

    uint8_t mag_config[] = {0x02, 0x00};  // continuous mag
    i2c_write_blocking(I2C_INST, MAG_ADDR, mag_config, 2, false);
}

// -------------------- SENSOR READINGS (POLLING) --------------------
bool read_accel_raw(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az) {
    uint8_t acc_data[6];
    uint8_t reg = 0x28 | 0x80;  // enable auto-increment
    
    // Use polling I2C write
    int result = i2c_write_blocking(I2C_INST, ACC_ADDR, &reg, 1, true);
    if (result != 1) return false;
    
    // Use polling I2C read  
    result = i2c_read_blocking(I2C_INST, ACC_ADDR, acc_data, 6, false);
    if (result != 6) return false;

    *raw_ax = (int16_t)((acc_data[1] << 8) | acc_data[0]);
    *raw_ay = (int16_t)((acc_data[3] << 8) | acc_data[2]);
    *raw_az = (int16_t)((acc_data[5] << 8) | acc_data[4]);
    return true;
}

bool read_mag_raw(int16_t *mx, int16_t *my) {
    uint8_t mag_data[6];
    
    // Use polling I2C write
    int result = i2c_write_blocking(I2C_INST, MAG_ADDR, (uint8_t[]){0x03}, 1, true);
    if (result != 1) return false;
    
    // Use polling I2C read
    result = i2c_read_blocking(I2C_INST, MAG_ADDR, mag_data, 6, false);
    if (result != 6) return false;

    *mx = (int16_t)((mag_data[0] << 8) | mag_data[1]);
    *my = (int16_t)((mag_data[2] << 8) | mag_data[3]);
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

// -------------------- CORRECTED GRAVITY REMOVAL & FRAME TRANSFORMATION --------------------
void remove_gravity_and_transform_frame(float *ax, float *ay, float *az, float pitch, float roll) {
    // Convert angles to radians
    float pitch_rad = pitch * M_PI / 180.0f;
    float roll_rad = roll * M_PI / 180.0f;
    
    // Calculate the gravity vector in the sensor's frame
    float gravity_x = 1000.0f * sinf(pitch_rad);
    float gravity_y = -1000.0f * sinf(roll_rad) * cosf(pitch_rad);
    float gravity_z = 1000.0f * cosf(roll_rad) * cosf(pitch_rad);
    
    // Remove gravity from the raw sensor readings
    float ax_no_grav = *ax - gravity_x;
    float ay_no_grav = *ay - gravity_y;
    float az_no_grav = *az - gravity_z;
    
    // Now rotate the acceleration vector to the horizontal (earth) frame
    float cos_p = cosf(pitch_rad);
    float sin_p = sinf(pitch_rad);
    float cos_r = cosf(roll_rad);
    float sin_r = sinf(roll_rad);
    
    // Inverse rotation matrix (transpose of the forward rotation matrix)
    float x_rot = ax_no_grav;
    float y_rot = ay_no_grav * cos_r + az_no_grav * sin_r;
    float z_rot = -ay_no_grav * sin_r + az_no_grav * cos_r;
    
    // Then, undo pitch (around sensor's Y axis)
    *ax = x_rot * cos_p - z_rot * sin_p;
    *ay = y_rot;
    *az = x_rot * sin_p + z_rot * cos_p;
}

void calculate_orientation(int16_t ax, int16_t ay, int16_t az, float *pitch, float *roll) {
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    *roll = atan2f(ay, az) * 180.0f / M_PI;
}

// -------------------- HEADING CALCULATION --------------------
float calculate_heading(int16_t mx, int16_t my) {
    float heading = atan2f(my, mx) * 180.0f / M_PI;
    if (heading < 0) heading += 360.0f;
    return heading;
}

// -------------------- COMPUTE DATA (CORRECTED) --------------------
// -------------------- COMPUTE DATA (CORRECTED) --------------------
void compute_and_print_data(int16_t ax, int16_t ay, int16_t az) {
    int16_t ax_filtered = apply_filter(ax, ax_history, &filter_index);
    int16_t ay_filtered = apply_filter(ay, ay_history, &filter_index);
    int16_t az_filtered = apply_filter(az, az_history, &filter_index);

    int16_t mx, my;
    if (!read_mag_raw(&mx, &my)) {
        printf(" MAG_FAILED\n");
        return;
    }

    float heading = calculate_heading(mx, my);
    
    // Calculate orientation (pitch and roll)
    float pitch, roll;
    calculate_orientation(ax_filtered, ay_filtered, az_filtered, &pitch, &roll);

    // Convert to float for calculations
    float ax_float = (float)ax_filtered;
    float ay_float = (float)ay_filtered; 
    float az_float = (float)az_filtered;

    // Remove gravity and transform to horizontal frame
    float forward_accel = ax_float;
    float lateral_accel = ay_float;
    float vertical_accel = az_float;
    remove_gravity_and_transform_frame(&forward_accel, &lateral_accel, &vertical_accel, pitch, roll);

    // Calculate motion metrics
    float total_accel = sqrtf(forward_accel*forward_accel + lateral_accel*lateral_accel + vertical_accel*vertical_accel);
    float accel_change = fabsf(total_accel - last_total_accel);
    last_total_accel = total_accel;

    // Using your original thresholds
    bool is_jerky = (accel_change > 200);

    // Print IMU data in compact format (no newline)
    printf(" H:%4.0f° P:%4.0f° R:%4.0f°", heading, pitch, roll);
    if (is_jerky) printf(" [JERKY]");
    else printf("         ");  // Padding to keep alignment
}

// -------------------- IMU TASK FUNCTION --------------------
void compute_and_print_imu_data(void) {
    int16_t ax, ay, az;
    if (read_accel_raw(&ax, &ay, &az)) {
        compute_and_print_data(ax, ay, az);
    }
}

void imu_task(__unused void *params) {
    imu_init();
    
    printf("IMU system initialized (POLLING MODE - no I2C interrupts)\n");
    
    while(true) {
        int16_t ax, ay, az;
        if (read_accel_raw(&ax, &ay, &az)) {
            compute_and_print_imu_data();
        } else {
            printf(" IMU_READ_FAIL ");  // Show when IMU fails to read
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz update rate
    }
}