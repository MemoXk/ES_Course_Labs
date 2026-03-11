#include "TIMER0_interface.h"
#include "TIMER0_private.h"
#include "TIMER0_config.h"
#include "../../SERVICES/BIT_MATH.h"

static void (*TIMER0_Callback)(void) = NULL_PTR;
static u8 ovf_count = 0;

void TIMER0_Init(void)
{
    /* Internal clock source (Fosc/4) — clear T0CS */
    CLR_BIT(OPTION_REG, OPTION_T0CS);

    /* Assign prescaler to Timer0 — clear PSA */
    CLR_BIT(OPTION_REG, OPTION_PSA);

    /* Set prescaler bits PS2:PS0 (bits 2:0) without disturbing other bits */
    OPTION_REG = (OPTION_REG & 0xF8u) | (TIMER0_PRESCALER & 0x07u);

    /* Start Timer0 from 0 (first period is a full overflow) */
    TMR0 = 0;

    /* Clear flag, enable Timer0 overflow interrupt and global interrupts */
    CLR_BIT(INTCON, INTCON_T0IF);
    SET_BIT(INTCON, INTCON_T0IE);
    SET_BIT(INTCON, INTCON_GIE);
}

void TIMER0_Reset(void)
{
    CLR_BIT(INTCON, INTCON_T0IF);
    ovf_count = 0;
    TMR0 = 0;
}

void TIMER0_SetCallback(void (*ptr)(void))
{
    TIMER0_Callback = ptr;
}

/*  Mixed-overflow strategy — called from interrupt() in APP/main.c.
    ISR fires after each Timer0 overflow; we track how many have occurred:
      ovf_count 1..29 : reload TMR0=0    (full 256-count overflow, 32.768 ms)
      ovf_count == 30  : reload TMR0=124 (partial 132-count overflow, 16.896 ms)
      ovf_count == 31  : 30×256+132 = 7812 counts × 128 µs ≈ 1 second → fire callback */
void TIMER0_IRQHandler(void)
{
    if (!GET_BIT(INTCON, INTCON_T0IF))
        return;                               /* not a Timer0 overflow */

    ovf_count++;

    if (ovf_count < TIMER0_OVF_FULL_COUNT)
    {
        TMR0 = 0;                             /* next: full overflow   */
    }
    else if (ovf_count == TIMER0_OVF_FULL_COUNT)
    {
        TMR0 = TIMER0_PRELOAD;                /* next: partial overflow */
    }
    else
    {
        ovf_count = 0;
        TMR0 = 0;                             /* restart cycle          */
        if (TIMER0_Callback != NULL_PTR)
            TIMER0_Callback();                /* ~1 second has elapsed  */
    }

    CLR_BIT(INTCON, INTCON_T0IF);
}
