#include "LED_interface.h"
#include "../../MCAL/GPIO/GPIO_interface.h"
#include "../../MCAL/GPIO/GPIO_private.h"   /* PORTA..PORTE latch macros for XOR toggle */

void LED_Init(u8 Port, u8 Pin)
{
    GPIO_SetPinDirection(Port, Pin, GPIO_OUTPUT);
}

void LED_On(u8 Port, u8 Pin)
{
    GPIO_SetPinValue(Port, Pin, GPIO_HIGH);
}

void LED_Off(u8 Port, u8 Pin)
{
    GPIO_SetPinValue(Port, Pin, GPIO_LOW);
}

/* Toggle by XORing the output latch register directly.
   This avoids calling GPIO_GetPinValue, which would cause a mikroC
   reentrancy error when LED_Toggle is called from both the ISR and
   main context. XOR on PORTx also avoids the read-modify-write
   noise hazard on PIC I/O pins. */
void LED_Toggle(u8 Port, u8 Pin)
{
    switch(Port)
    {
        case GPIO_PORTA: PORTA ^= (1u << Pin); break;
        case GPIO_PORTB: PORTB ^= (1u << Pin); break;
        case GPIO_PORTC: PORTC ^= (1u << Pin); break;
        case GPIO_PORTD: PORTD ^= (1u << Pin); break;
        case GPIO_PORTE: PORTE ^= (1u << Pin); break;
        default: break;
    }
}