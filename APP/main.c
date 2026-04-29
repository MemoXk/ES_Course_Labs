/*
 * main.c — Motor test entry point
 *
 * Device : PIC16F877A
 * Clock  : 20 MHz external crystal (HS oscillator)
 * MikroC project settings: FOSC=HS, WDTE=OFF, PWRTE=ON, LVP=OFF
 *
 * Delay note: Delay_ms() max reliable value at 20 MHz = 500 ms.
 * All 2-second delays are implemented as 4 x Delay_ms(500).
 */

#include "motor_test.h"

int main(void)
{
    MOTOR_Test();   /* loops forever: Forward -> Stop -> Backward -> Stop */

    return 0;
}
