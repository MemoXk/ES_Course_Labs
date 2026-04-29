#include "motor_test.h"
#include "../HAL/MOTOR/MOTOR_interface.h"

/* 2-second delay helper — __delay_ms(2000) resets PIC at 20MHz */
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

    while(1)
    {
        MOTOR_Forward();
        delay_2s();

        MOTOR_Stop();
        delay_2s();

        MOTOR_Backward();
        delay_2s();

        MOTOR_Stop();
        delay_2s();
    }
}
