#include "motor_test.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../MCAL/PWM/PWM_Interface.h"

/*
 * Motor speed test at 25% PWM.
 *
 * Wiring:
 *   RD0 → IN1   (motor A direction)
 *   RD1 → IN2   (motor A direction)
 *   RD2 → IN3   (motor B direction)
 *   RD3 → IN4   (motor B direction)
 *   RC2 → ENA AND ENB on L298N  (one PWM, both motors get same duty)
 *
 * IMPORTANT: remove the ENA and ENB jumper caps on the L298N
 *            before connecting RC2 — otherwise the enables stay
 *            tied to +5V and PWM has no effect.
 *
 * Sequence (loops forever):
 *   forward 2s → backward 2s → left 2s → right 2s → stop 2s
 */

static void delay_2s(void)
{
    __delay_ms(500);
    __delay_ms(500);
    __delay_ms(500);
    __delay_ms(500);
}

void MOTOR_Test(void)
{
    MOTOR_Init();

    /* PWM on RC2 → ENA/ENB at 25% */
    PWM_Init();
    PWM_SetDutyCycle(25);
    PWM_Start();

    while(1)
    {
        MOTOR_Forward();
        delay_2s();

        MOTOR_Backward();
        delay_2s();

        MOTOR_TurnLeft();
        delay_2s();

        MOTOR_TurnRight();
        delay_2s();

        MOTOR_Stop();
        delay_2s();
    }
}
