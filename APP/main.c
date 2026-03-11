/* ================================================================
 * Lab Quiz – CIE 408/349 Embedded Systems, Spring 2026
 * ================================================================
 * Behaviour (from circuit diagram):
 *   LED1 toggles via Timer0 every 2.5 s
 *   SW1  toggles LED2 on each press  (INT0 / RB0, falling edge)
 *   SW2  resets the Timer0 cycle when pressed
 *   After 5 external interrupts: LED1 stops, LED2 stays ON permanently
 *
 * Pin assignments (PIC16F/18F877A):
 *   LED1 (RED)  → RD0  (PORTD, PIN0)
 *   LED2 (BLUE) → RD1  (PORTD, PIN1)
 *   SW1         → RB0  (INT0 – configured inside EXT_INT0_Init)
 *   SW2         → RA4  (PORTA, PIN4, active HIGH / pull-down R4)
 * ================================================================ */

#include "../MCAL/MCAL_drivers.h"
#include "../HAL/HAL_drivers.h"

/* ── Pin definitions ──────────────────────────────────────────── */
#define LED1_PORT   GPIO_PORTD
#define LED1_PIN    GPIO_PIN0

#define LED2_PORT   GPIO_PORTD
#define LED2_PIN    GPIO_PIN1

/* SW1 = RB0 / INT0 (direction set by EXT_INT0_Init automatically) */
#define SW2_PORT    GPIO_PORTA
#define SW2_PIN     GPIO_PIN4

/* ── Shared volatile state ────────────────────────────────────── */
volatile unsigned char ext_counter = 0;
volatile unsigned char led1_active = 1;  /* flag to control LED1 blinking */

/* ── HAL helper ── SW2 reads HIGH when pressed (pull-down R4) ─── */
static u8 SW2_Read(void)
{
    return SWITCH_GetState(SW2_PORT, SW2_PIN);
}

/* ── Timer0 callback: fires every 2.5 s ──────────────────────── */
static void Timer0_Callback(void)
{
    if (led1_active)
        LED_Toggle(LED1_PORT, LED1_PIN);
}

/* ── EXT_INT0 callback: fires on each SW1 press ──────────────── */
static void SW1_Callback(void)
{
    /* toggle LED2 on every press */
    LED_Toggle(LED2_PORT, LED2_PIN);
    ext_counter++;

    if (ext_counter >= 4)       /* skeleton threshold: stops at 4th press */
    {
        led1_active = 0;        /* stop LED1 blinking */
        LED_Off(LED1_PORT, LED1_PIN);
        LED_On(LED2_PORT, LED2_PIN);   /* LED2 ON permanently */
    }
}

/* ── Unified ISR dispatcher ───────────────────────────────────── */
void interrupt()
{
    TIMER0_IRQHandler();
    EXT_INT0_IRQHandler();
}

/* ── Entry point ──────────────────────────────────────────────── */
void main()
{
    /* do not forget to initialize clocks, GPIOs, timers */
    LED_Init(LED1_PORT, LED1_PIN);
    LED_Init(LED2_PORT, LED2_PIN);
    LED_Off(LED1_PORT, LED1_PIN);
    LED_Off(LED2_PORT, LED2_PIN);

    SWITCH_Init(SW2_PORT, SW2_PIN);

    TIMER0_Init();                          /* initialize timer0 (enables interrupt) */
    TIMER0_SetCallback(Timer0_Callback);

    EXT_INT0_Init();                        /* INT0 init – falling edge on RB0 */
    EXT_INT0_SetCallback(SW1_Callback);

    while (1)
    {
        /* SW2: active HIGH (pull-down R4) → reset Timer0 cycle */
        if (SW2_Read())
        {
            TIMER0_Reset();
        }
    }
}
