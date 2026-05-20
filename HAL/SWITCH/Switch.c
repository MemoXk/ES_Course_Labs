#include "Switch_interface.h"

void Switch_Init(u8 port, u8 pin, u8 active)
{
    (void)active; /* Active polarity is stateless. */
    GPIO_SetPinDirection(port, pin, GPIO_INPUT);
}

u8 Switch_GetState(u8 port, u8 pin, u8 active)
{
    u8 raw = GPIO_GetPinValue(port, pin);

    if(active == SWITCH_ACTIVE_LOW)
    {
        return (raw == GPIO_LOW) ? SWITCH_PRESSED : SWITCH_RELEASED;
    }

    return (raw == GPIO_HIGH) ? SWITCH_PRESSED : SWITCH_RELEASED;
}

u8 Switch_ReadRaw(u8 port, u8 pin)
{
    return GPIO_GetPinValue(port, pin);
}

u8 Switch_IsPressed(u8 port, u8 pin, u8 active)
{
    return (Switch_GetState(port, pin, active) == SWITCH_PRESSED) ? 1U : 0U;
}
