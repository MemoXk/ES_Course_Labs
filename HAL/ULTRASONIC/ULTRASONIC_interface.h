#ifndef ULTRASONIC_INTERFACE_H
#define ULTRASONIC_INTERFACE_H

#include "ULTRASONIC_private.h"
#include "ULTRASONIC_config.h"
#include "../../SERVICES/STD_TYPES.h"
#include "../../MCAL/TIMER_1/TIMER_1_Interface.h"

/*
 * Configure all TRIG pins as output (LOW) and all ECHO pins as input.
 * Must be called once before any ULTRASONIC_GetDistance() call.
 */
void ULTRASONIC_Init(void);

/*
 * Configure only one sensor's TRIG/ECHO pins.
 * Use this for staged hardware bring-up to avoid driving unused sensors.
 */
void ULTRASONIC_InitSensor(u8 sensor_id);

/*
 * Trigger one measurement on the selected sensor and return distance in cm.
 * sensor_id: ULTRASONIC_FRONT / BACK / LEFT / RIGHT
 * Returns ULTRASONIC_NO_OBJ (999) if no echo received within range.
 * Uses MCAL Timer1 internally; do not share Timer1 while measuring.
 */
u16 ULTRASONIC_GetDistance(u8 sensor_id);

/*
 * Trigger one measurement and return detailed status for diagnostics.
 * status codes:
 *   'O' = ok, 'H' = stale echo high, 'N' = no rising edge,
 *   'T' = echo high timed out, 'S' = too short/glitch, 'I' = invalid sensor.
 */
u16 ULTRASONIC_ReadDetailed(u8 sensor_id, u8* status, u16* pulse_ticks);

/*
 * Helpers for the app-level three-sample median filter.
 */
void ULTRASONIC_AddValidSample(u16 samples[], u8* count, u16 sample_cm, u8 status);
u16  ULTRASONIC_MedianOrNoEcho(u16 samples[], u8 count);

#endif
