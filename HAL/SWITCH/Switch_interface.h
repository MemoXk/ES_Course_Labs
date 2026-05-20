#ifndef SWITCH_INTERFACE_H
#define SWITCH_INTERFACE_H

#include "../../SERVICES/STD_TYPES.h"
#include "../../MCAL/GPIO/GPIO_interface.h"
#include "Switch_private.h"
#include "Switch_config.h"

#define SWITCH_PRESSED      1U
#define SWITCH_RELEASED     0U

void Switch_Init(u8 port, u8 pin, u8 active);
u8   Switch_GetState(u8 port, u8 pin, u8 active);
u8   Switch_ReadRaw(u8 port, u8 pin);
u8   Switch_IsPressed(u8 port, u8 pin, u8 active);

#endif
