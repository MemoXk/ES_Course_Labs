#ifndef INTERRUPT_MANAGER_CONFIG_H
#define INTERRUPT_MANAGER_CONFIG_H

/*
 * Active manual-control firmware only uses the UART RX interrupt.
 * Timer0 and RB0/INT dispatch paths stay available for lab tests by flipping
 * these config switches, but excluding them here keeps the PIC16 hardware stack
 * analysis focused on the running application.
 */
#define INTERRUPT_MANAGER_UART_RX_ENABLE   1U
#define INTERRUPT_MANAGER_TIMER0_ENABLE    0U
#define INTERRUPT_MANAGER_EXT_INT_ENABLE   0U

#endif
