/*
 * pwm_test.c
 *
 * Hardware PWM breathing test on RC2 (CCP1).
 * Connect LED + resistor between RC2 and GND.
 *
 * Behaviour:
 *   duty 0% → 100%  (LED fades in,  ~2 seconds)
 *   duty 100% → 0%  (LED fades out, ~2 seconds)
 *   repeats forever
 *
 * Step size : 1%
 * Step delay: 20 ms  → 100 steps × 20 ms = 2 s per ramp
 *
 * Pin  : RC2 (pin 17 on PIC16F877A)
 * Freq : 8 kHz (from PWM_Config.h)
 */

#include "../MCAL/PWM/PWM_Interface.h"
#include "pwm_test.h"

void PWM_Test(void)
{
    u8 duty;

    PWM_Init();
    PWM_Start();

    while(1)
    {
        /* Fade in: 0% → 100% */
        for(duty = 0; duty <= 100U; duty++)
        {
            PWM_SetDutyCycle(duty);
            __delay_ms(20);
        }

        /* Fade out: 100% → 0% */
        for(duty = 100U; duty > 0U; duty--)
        {
            PWM_SetDutyCycle(duty);
            __delay_ms(20);
        }
        PWM_SetDutyCycle(0);
        __delay_ms(20);
    }
}
