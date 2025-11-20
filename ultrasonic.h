/**
 * ultrasonic.h - Header file for HC-SR04 ultrasonic distance sensor functionality.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"

// Pin definitions adjusted to match the current wiring:
// TRIG connected to GP0, ECHO connected to GP1.
// These can be changed later if wiring is updated.
#define TRIG_PIN 0
#define ECHO_PIN 1

// Initializes GPIO pins used by the ultrasonic sensor
void ultrasonic_init(void);

// Sends trigger pulse and measures the distance in centimeters
float ultrasonic_get_distance_cm(void);

// Add this function prototype to ultrasonic.h
void ultrasonic_trigger_measurement(void);
bool ultrasonic_detect_obstacle_fast(void);
#endif
