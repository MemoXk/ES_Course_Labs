/*
 * Device : PIC16F877A @ 20 MHz HS oscillator
 * Compiler: MPLAB X + XC8
 *
 * Behavior: drive forward, front sensor < 20cm -> stop -> backward 2s -> forward
 *           extra sensor (RD4/RD5) < 20cm -> stop 10s -> forward
 * Motor pins : RD0-RD3
 * Front US   : RB0 TRIG, RB1 ECHO
 * Extra US   : RD4 TRIG, RD5 ECHO
 */

// CONFIG
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config LVP  = OFF

#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../HAL/ULTRASONIC/ULTRASONIC_interface.h"

static void delay_2s(void)
{
    __delay_ms(500); __delay_ms(500);
    __delay_ms(500); __delay_ms(500);
}

static void delay_10s(void)
{
    __delay_ms(500); __delay_ms(500); __delay_ms(500); __delay_ms(500);
    __delay_ms(500); __delay_ms(500); __delay_ms(500); __delay_ms(500);
    __delay_ms(500); __delay_ms(500); __delay_ms(500); __delay_ms(500);
    __delay_ms(500); __delay_ms(500); __delay_ms(500); __delay_ms(500);
    __delay_ms(500); __delay_ms(500); __delay_ms(500); __delay_ms(500);
}

int main(void)
{
    MOTOR_Init();
    ULTRASONIC_Init();
    MOTOR_Forward();

    while(1)
    {
        if(ULTRASONIC_GetDistance(ULTRASONIC_FRONT) < ULTRASONIC_STOP_THRESHOLD)
        {
            MOTOR_Stop();
            delay_2s();
            MOTOR_Backward();
            delay_2s();
            MOTOR_Stop();
            delay_2s();
            MOTOR_Forward();
        }

        if(ULTRASONIC_GetDistance(ULTRASONIC_EXTRA) < ULTRASONIC_STOP_THRESHOLD)
        {
            MOTOR_Stop();
            delay_10s();
            MOTOR_Forward();
        }
    }

    return 0;
}
