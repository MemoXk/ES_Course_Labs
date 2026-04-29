#ifndef SOFT_PWM_CONFIG_H
#define SOFT_PWM_CONFIG_H

#include "../../MCAL/GPIO/GPIO_interface.h"

/* Output pin — RD0 (free now that motors moved to RD4-RD7) */
#define SOFT_PWM_PORT       GPIO_PORTD
#define SOFT_PWM_PIN        GPIO_PIN0

/* Duty cycle 0-100 % */
#define SOFT_PWM_DUTY       25u

/* Period in microseconds — 10000us = 100 Hz */
#define SOFT_PWM_PERIOD_US  10000u

#endif
