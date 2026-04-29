/*
 * main.c — Obstacle avoidance test
 *
 * Device : PIC16F877A
 * Clock  : 20 MHz external crystal (HS oscillator)
 * MikroC project settings: FOSC=HS, WDTE=OFF, PWRTE=ON, LVP=OFF
 *
 * Hardware: remove ENA/ENB jumpers from L298N,
 *           connect RC2 (CCP1 PWM) to both ENA and ENB.
 */

#include "motor_obstacle_test.h"

int main(void)
{
    MOTOR_OBSTACLE_Test();  /* drives at 25% PWM, stops on obstacle < 20 cm */

    return 0;
}
