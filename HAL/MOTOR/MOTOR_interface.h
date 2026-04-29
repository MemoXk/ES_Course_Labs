#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include "MOTOR_private.h"
#include "MOTOR_config.h"
#include "../../SERVICES/STD_TYPES.h"

void MOTOR_Init(void);
void MOTOR_Forward(void);
void MOTOR_Backward(void);
void MOTOR_TurnLeft(void);
void MOTOR_TurnRight(void);
void MOTOR_Stop(void);

#endif
