/*
 * Device  : PIC16F877A @ 20 MHz HS oscillator
 * Compiler: MPLAB X + XC8
 *
 * Active test: motor sequence at 25% PWM speed.
 *   forward 2s → backward 2s → left 2s → right 2s → stop 2s, repeats forever.
 *
 * Wiring:
 *   RD0..RD3 → L298N IN1..IN4
 *   RC2      → L298N ENA + ENB (jumper caps REMOVED)
 */

// CONFIG
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config LVP  = OFF

#include "motor_test.h"

int main(void)
{
    MOTOR_Test();
    return 0;
}
