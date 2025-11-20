/**
 * imu_raw_demo.h - Header file for raw IMU sensor interface and functions.
 */

#ifndef IMU_RAW_DEMO_H
#define IMU_RAW_DEMO_H

#include <stdbool.h>
#include <stdint.h>

// Function declarations
void  imu_init(void);
bool  read_accel_raw(int16_t* raw_ax, int16_t* raw_ay, int16_t* raw_az);
bool  read_mag_raw(int16_t* mx, int16_t* my, int16_t* mz); // Add mz parameter
void  compute_and_print_imu_data(void);
void  imu_task(void* params);
float calculate_simple_yaw(int16_t mx, int16_t my);
float get_heading_fast(float* direction_str);
void  reset_heading_reference(void);
void  simple_calibration(void);

#endif // IMU_RAW_DEMO_H
