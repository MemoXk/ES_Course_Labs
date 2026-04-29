#include "SOFT_PWM_interface.h"

void SOFT_PWM_Init(void)
{
    GPIO_SetPinDirection(SOFT_PWM_PORT, SOFT_PWM_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(SOFT_PWM_PORT, SOFT_PWM_PIN, GPIO_LOW);
}

void SOFT_PWM_Tick(void)
{
    GPIO_SetPinValue(SOFT_PWM_PORT, SOFT_PWM_PIN, GPIO_HIGH);
    Delay_us(SOFT_PWM_HIGH_US);
    GPIO_SetPinValue(SOFT_PWM_PORT, SOFT_PWM_PIN, GPIO_LOW);
    Delay_us(SOFT_PWM_LOW_US);
}

void SOFT_PWM_Stop(void)
{
    GPIO_SetPinValue(SOFT_PWM_PORT, SOFT_PWM_PIN, GPIO_LOW);
}
