#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "../../MCAL/GPIO/GPIO_interface.h"

/* All four L298N control lines are on PORTD */
#define MOTOR_PORT      GPIO_PORTD

#define MOTOR_IN1       GPIO_PIN0   /* Left  side forward  */
#define MOTOR_IN2       GPIO_PIN1   /* Left  side backward */
#define MOTOR_IN3       GPIO_PIN2   /* Right side forward  */
#define MOTOR_IN4       GPIO_PIN3   /* Right side backward */

#endif
