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

// Initializes GPIO pins used by the ultrasonic sensor
void ultrasonic_init(void);

// Sends trigger pulse and measures the distance in centimeters
float ultrasonic_get_distance_cm(void);

#endif
