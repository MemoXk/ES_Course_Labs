/*
 * pwm_test.c
 *
 * Hardware PWM breathing test on RC2 (CCP1) +
 * heartbeat blink on RD0 (proves the chip is running).
 *
 * Wiring:
 *   RC2 (pin 17) ──[330Ω]──[LED]── GND   (PWM breathing LED)
 *   RD0 (pin 19) ──[330Ω]──[LED]── GND   (heartbeat — toggles every step)
 *
 * Expected:
 *   - RD0 LED: blinks rapidly (toggles every 20ms during fade) → chip is alive
 *   - RC2 LED: smooth fade in/out                              → PWM works
 *
 * If RD0 blinks but RC2 stays dark → RC2 is damaged hardware.
 * If neither blinks → chip not running (check power/oscillator/MCLR).
 */

#include "../MCAL/PWM/PWM_Interface.h"
#include "../MCAL/GPIO/GPIO_interface.h"
#include "../MCAL/DELAY/DELAY_Interface.h"
#include "pwm_test.h"

void PWM_Test(void)
{
    u8 duty;
    u8 hb = 0;

    /* Heartbeat LED on RD0 */
    GPIO_SetPinDirection(GPIO_PORTD, GPIO_PIN0, GPIO_OUTPUT);
    GPIO_SetPinValue(GPIO_PORTD, GPIO_PIN0, GPIO_LOW);

    PWM_Init();
    PWM_Start();

    while(1)
    {
        /* Fade in: 0% → 100% */
        for(duty = 0; duty <= 100U; duty++)
        {
            PWM_SetDutyCycle(duty);
            hb ^= 1;
            GPIO_SetPinValue(GPIO_PORTD, GPIO_PIN0, hb ? GPIO_HIGH : GPIO_LOW);
            DELAY_ms(20U);
        }

        /* Fade out: 100% → 0% */
        for(duty = 100U; duty > 0U; duty--)
        {
            PWM_SetDutyCycle(duty);
            hb ^= 1;
            GPIO_SetPinValue(GPIO_PORTD, GPIO_PIN0, hb ? GPIO_HIGH : GPIO_LOW);
            DELAY_ms(20U);
        }
        PWM_SetDutyCycle(0);
        DELAY_ms(20U);
    }
}
