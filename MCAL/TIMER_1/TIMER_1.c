#include "TIMER_1_Interface.h"

void TIMER1_Init(void)
{
    /*
     * Internal clock, oscillator disabled, configured prescaler, stopped.
     * T1CON bit 0 (TMR1ON) is left clear until TIMER1_Start().
     */
    T1CON = (u8)(TIMER1_PRESCALER & TIMER1_PRESCALE_MASK);
}

void TIMER1_Reset(void)
{
    TMR1H = 0u;
    TMR1L = 0u;
}

void TIMER1_Start(void)
{
    T1CON = (u8)((T1CON & TIMER1_CONTROL_MASK) | (TIMER1_PRESCALER & TIMER1_PRESCALE_MASK));
    CLR_BIT(T1CON, TMR1CS_BIT);
    CLR_BIT(T1CON, T1OSCEN_BIT);
    SET_BIT(T1CON, TMR1ON_BIT);
}

void TIMER1_Stop(void)
{
    CLR_BIT(T1CON, TMR1ON_BIT);
}

u16 TIMER1_GetValue(void)
{
    u8 high;
    u8 low;

    high = TMR1H;
    low  = TMR1L;

    return (u16)(((u16)high << 8) | (u16)low);
}

u8 TIMER1_GetHighByte(void)
{
    return TMR1H;
}
