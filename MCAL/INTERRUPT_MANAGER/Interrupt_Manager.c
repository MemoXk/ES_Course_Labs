#include "Interrupt_Manager_Interface.h"
#include "Interrupt_Manager_config.h"
#include "../USART/USART_Interface.h"
#include "../EXT_INT/EXT_INT_Interface.h"
#include "../TIMER_0/TIMER_0_Interface.h"
#include "../../SERVICES/BIT_MATH.h"

/*
 * Central interrupt dispatcher for PIC16F877A.
 *
 * PIC16F877A uses a single interrupt entry point.
 * Each peripheral's ISR function is called after checking its flag.
 * Flags are cleared inside each driver's ISR routine.
 */

INTERRUPT_MANAGER_ISR_ENTRY(isr)
{
#if INTERRUPT_MANAGER_UART_RX_ENABLE
    /* ---- UART RX interrupt (PIR1.RCIF) ---- */
    if(GET_BIT(PIR1, RCIF_BIT) && GET_BIT(PIE1, RCIE_BIT))
    {
        UART_ISR();
    }
#endif

#if INTERRUPT_MANAGER_TIMER0_ENABLE
    /* ---- Timer0 overflow interrupt (INTCON.T0IF) ---- */
    if(GET_BIT(INTCON, T0IF_BIT) && GET_BIT(INTCON, T0IE_BIT))
    {
        TIMER0_ISR();
    }
#endif

#if INTERRUPT_MANAGER_EXT_INT_ENABLE
    /* ---- External interrupt RB0/INT (INTCON.INTF) ---- */
    if(GET_BIT(INTCON, INTF_BIT) && GET_BIT(INTCON, INTE_BIT))
    {
        EXT_INT_ISR();
    }
#endif
}
