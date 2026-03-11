#include "../HAL/LED/LED_interface.h"
#include "../HAL/SWITCH/SWITCH_interface.h"
#include "../MCAL/TIMER0/TIMER0_interface.h"
#include "../MCAL/EXT_INT/EXT_INT_interface.h"
#include "../MCAL/GPIO/GPIO_interface.h"

/* Pin assignments
   LED1  : RB1  — toggles every 2.5 s via Timer0
   LED2  : RA4  — toggled by SW1 (INT0/RB0); stays ON after 5 presses
   SW1   : RB0  — INT0 external interrupt
   SW2   : RB4  — system reset (restarts Timer0 cycle)                    */
#define LED1_PORT   GPIO_PORTB
#define LED1_PIN    GPIO_PIN1
#define LED2_PORT   GPIO_PORTA
#define LED2_PIN    GPIO_PIN4
#define SW2_PORT    GPIO_PORTB
#define SW2_PIN     GPIO_PIN4

/* RA4 open-drain fix:
   RA4 on PIC16F877A is open-drain — it can only sink current, not source it.
   If LED2 is wired active-low (VDD -> R -> LED -> RA4):
     writing 0 (LOW)  = RA4 sinks current  = LED ON
     writing 1 (HIGH) = RA4 floating/open  = LED OFF
   In that case, uncomment the line below to invert LED2 logic.
   If LED2 is wired active-high (RA4 -> R -> LED -> GND), leave it commented. */
/* #define LED2_ACTIVE_LOW */

#ifdef LED2_ACTIVE_LOW
#  define LED2_On()     LED_Off(LED2_PORT, LED2_PIN)   /* write 0 = sinks = ON  */
#  define LED2_Off()    LED_On(LED2_PORT, LED2_PIN)    /* write 1 = open  = OFF */
#else
#  define LED2_On()     LED_On(LED2_PORT, LED2_PIN)
#  define LED2_Off()    LED_Off(LED2_PORT, LED2_PIN)
#endif
/* LED_Toggle works the same in both cases — just flips the bit */

static volatile u8 ext_counter = 0;  /* counts SW1 presses                */
static volatile u8 led1_active = 1;  /* 1 = LED1 blinks; 0 = stopped      */

/* Timer0 callback: fires every 2.5 s */
static void Timer0Callback(void)
{
    if (led1_active)
        LED_Toggle(LED1_PORT, LED1_PIN);
}

/* EXT_INT0 callback: SW1 pressed */
static void SW1Callback(void)
{
    LED_Toggle(LED2_PORT, LED2_PIN);
    ext_counter++;

    if (ext_counter >= 5)
    {
        led1_active = 0;   /* stop LED1 blinking      */
        LED2_On();         /* LED2 permanently ON     */
    }
}

/* Unified ISR dispatcher */
void interrupt()
{
    TIMER0_IRQHandler();
    EXT_INT0_IRQHandler();
}

void main()
{
    /* Initialize LEDs and SW2 */
    LED_Init(LED1_PORT, LED1_PIN);
    LED_Init(LED2_PORT, LED2_PIN);
    SWITCH_Init(SW2_PORT, SW2_PIN);

    /* Timer0: 2.5-second LED1 toggle */
    TIMER0_Init();
    TIMER0_SetCallback(Timer0Callback);
    TIMER0_EnableInterrupt();

    /* INT0: SW1 on RB0 toggles LED2, counts presses */
    EXT_INT0_Init();
    EXT_INT0_SetCallback(SW1Callback);

    while (1)
    {
        /* SW2: reset Timer0 cycle (system reset) */
        if (SWITCH_GetState(SW2_PORT, SW2_PIN) == SWITCH_PRESSED)
        {
            TIMER0_Reset();
        }
    }
}
