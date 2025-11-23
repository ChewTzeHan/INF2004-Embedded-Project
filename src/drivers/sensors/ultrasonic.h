//ultrasonic.h

/*
Header file defining pin assignments and function prototypes for the ultrasonic
 distance measurement module.

TRIG_PIN and ECHO_PIN specify which GPIO pins are used for sending the trigger signal
 and receiving the echo.

The function prototypes allow main.c to access the initialization and measurement routines.
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "pico/stdlib.h"

// Pin definitions adjusted to match the current wiring:
// TRIG connected to GP0, ECHO connected to GP1.
// These can be changed later if wiring is updated.
#define TRIG_PIN 0
#define ECHO_PIN 1

/**
 * @brief Configure TRIG and ECHO GPIO pins for the HC-SR04 sensor.
 *
 * Must be called once at startup before using any ultrasonic functions.
 */
void ultrasonic_init(void);

/**
 * @brief Trigger up to a few measurement attempts and return the measured distance.
 *
 * Converts the echo pulse width into distance in centimetres.
 * @return Measured distance in cm, or -1.0f if all attempts fail.
 */
float ultrasonic_get_distance_cm(void);

/**
 * @brief Send a single trigger pulse. Mostly used internally by higher-level helpers.
 */
void ultrasonic_trigger_measurement(void);

/**
 * @brief Fast boolean check used by obstacle avoidance logic.
 *
 * @return true if an obstacle is within the configured detection range.
 */
bool ultrasonic_detect_obstacle_fast(void);
#endif
