#ifndef MANUAL_CONTROL_CONFIG_H
#define MANUAL_CONTROL_CONFIG_H

#include "../MCAL/GPIO/GPIO_interface.h"
#include "../HAL/SWITCH/Switch_interface.h"

/* Unique boot identity printed after BOOT so the programmed hex is obvious. */
#define MANUAL_BUILD_ID "DIAG:BUILD_DRIVER_CLEANUP_50CM_20260520_A"

/* Seat-belt switch: buckled when RB0 reads high. */
#define MANUAL_SEATBELT_PORT    GPIO_PORTB
#define MANUAL_SEATBELT_PIN     GPIO_PIN0
#define MANUAL_SEATBELT_ACTIVE  SWITCH_ACTIVE_HIGH

/* LDR module: DO high means dark in the final wiring. */
#define MANUAL_LDR_DO_PORT      GPIO_PORTD
#define MANUAL_LDR_DO_PIN       GPIO_PIN4
#define MANUAL_LDR_ACTIVE       SWITCH_ACTIVE_HIGH
#define MANUAL_LDR_LED_PORT     GPIO_PORTD
#define MANUAL_LDR_LED_PIN      GPIO_PIN5

/* PWM throttle policy. */
#define MANUAL_DRIVE_START_DUTY 65U
#define MANUAL_DRIVE_MIN_DUTY   45U
#define MANUAL_DRIVE_WARN_DUTY  80U
#define MANUAL_DRIVE_MAX_DUTY   90U
#define MANUAL_DRIVE_STEP_DUTY  5U

/* Foreground command service limits. */
#define MANUAL_RX_DRAIN_LIMIT         8U
#define MANUAL_DELAY_SLICE_MS         10U
#define MANUAL_POST_TELEMETRY_TICKS   2U

#endif
