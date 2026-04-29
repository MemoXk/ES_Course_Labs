#include "motor_test.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../MCAL/PWM/PWM_Interface.h"
#include "../MCAL/GPIO/GPIO_interface.h"

/*
 * Motor speed test at 25% PWM, plus heartbeat LED.
 *
 * Wiring:
 *   RD0 → IN1   (motor A direction)
 *   RD1 → IN2   (motor A direction)
 *   RD2 → IN3   (motor B direction)
 *   RD3 → IN4   (motor B direction)
 *   RC2 → ENA AND ENB on L298N  (one PWM, both motors get same duty)
 *   RB0 → [330Ω] → LED → GND     (heartbeat: blinks at ~1 Hz while running)
 *
 * IMPORTANT: remove the ENA and ENB jumper caps on the L298N
 *            before connecting RC2 — otherwise the enables stay
 *            tied to +5V and PWM has no effect.
 *
 * Sequence (loops forever):
 *   forward 2s → backward 2s → left 2s → right 2s → stop 2s
 *
 * Heartbeat: RB0 LED toggles every 500 ms while the test runs.
 * If the LED is dark or stuck on/off → the chip isn't running.
 */

#define HB_PORT   GPIO_PORTB
#define HB_PIN    GPIO_PIN0

static u8 hb_state = 0;

static void hb_toggle(void)
{
    hb_state ^= 1U;
    GPIO_SetPinValue(HB_PORT, HB_PIN, hb_state ? GPIO_HIGH : GPIO_LOW);
}

/* Wait 2 seconds total, blinking heartbeat every 500 ms */
static void delay_2s(void)
{
    __delay_ms(500); hb_toggle();
    __delay_ms(500); hb_toggle();
    __delay_ms(500); hb_toggle();
    __delay_ms(500); hb_toggle();
}

void MOTOR_Test(void)
{
    /* Heartbeat LED */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    MOTOR_Init();

    /* PWM on RC2 → ENA/ENB at 25% */
    PWM_Init();
    PWM_SetDutyCycle(65);
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
