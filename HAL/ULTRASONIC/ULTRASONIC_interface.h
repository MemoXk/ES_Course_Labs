#ifndef ULTRASONIC_INTERFACE_H
#define ULTRASONIC_INTERFACE_H

#include "ULTRASONIC_private.h"
#include "ULTRASONIC_config.h"
#include "../../SERVICES/STD_TYPES.h"

/*
 * Configure all TRIG pins as output (LOW) and all ECHO pins as input.
 * Must be called once before any ULTRASONIC_GetDistance() call.
 */
void ULTRASONIC_Init(void);

/*
 * Trigger one measurement on the selected sensor and return distance in cm.
 * sensor_id: ULTRASONIC_FRONT / BACK / LEFT / RIGHT
 * Returns ULTRASONIC_NO_OBJ (999) if no echo received within range.
 * Uses Timer1 internally — do not use Timer1 elsewhere while calling this.
 */
u16 ULTRASONIC_GetDistance(u8 sensor_id);

#endif
