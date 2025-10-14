#ifndef IMU_RAW_DEMO_H
#define IMU_RAW_DEMO_H

#include <stdint.h>
#include <stdbool.h>

// Function declarations
void imu_init(void);
bool read_accel_raw(int16_t *raw_ax, int16_t *raw_ay, int16_t *raw_az);
bool read_mag_raw(int16_t *mx, int16_t *my);
void compute_and_print_imu_data(void);
void imu_task(void *params);  // ADD THIS LINE

#endif // IMU_RAW_DEMO_H