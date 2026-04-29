#include "../USART/USART_Interface.h"
#include "../EXT_INT/EXT_INT_Interface.h"
#include "../TIMER_0/TIMER_0_Interface.h"
#include "../../SERVICES/BIT_MATH.h"

/*
 * Central interrupt dispatcher for PIC16F877A.
 *
 * MikroC uses a single void interrupt() entry point.
 * Each peripheral's ISR function is called after checking its flag.
 * Flags are cleared inside each driver's ISR routine.
 */

void __interrupt() isr(void)
{
    /* ---- UART RX interrupt (PIR1.RCIF) ---- */
    if(GET_BIT(PIR1, RCIF_BIT))
    {
        UART_ISR();
    }

    /* ---- Timer0 overflow interrupt (INTCON.T0IF) ---- */
    if(GET_BIT(INTCON, T0IF_BIT))
    {
        TIMER0_ISR();
    }

    /* ---- External interrupt RB0/INT (INTCON.INTF) ---- */
    if(GET_BIT(INTCON, INTF_BIT))
    {
        EXT_INT_ISR();
    }
}