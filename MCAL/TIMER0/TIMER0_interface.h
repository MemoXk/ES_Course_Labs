#ifndef TIMER0_INTERFACE_H
#define TIMER0_INTERFACE_H

#include "../../SERVICES/STD_TYPES.h"

/* Prescaler options — pass one of these as TIMER0_PRESCALER in TIMER0_config.h */
#define TIMER0_PRESCALER_2      0x00u
#define TIMER0_PRESCALER_4      0x01u
#define TIMER0_PRESCALER_8      0x02u
#define TIMER0_PRESCALER_16     0x03u
#define TIMER0_PRESCALER_32     0x04u
#define TIMER0_PRESCALER_64     0x05u
#define TIMER0_PRESCALER_128    0x06u
#define TIMER0_PRESCALER_256    0x07u

void TIMER0_Init(void);
void TIMER0_EnableInterrupt(void); /* enable Timer0 overflow interrupt + GIE    */
void TIMER0_Reset(void);           /* restart the overflow-count cycle, TMR0=0  */
void TIMER0_SetCallback(void (*ptr)(void));
void TIMER0_IRQHandler(void);      /* call from interrupt() dispatcher in main.c */

#endif
