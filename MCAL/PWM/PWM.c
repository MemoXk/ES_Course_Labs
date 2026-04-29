#include "PWM_Interface.h"

void PWM_Init(void)
{
    /* Configure CCP2 pin (RC1) as output */
    GPIO_SetPinDirection(GPIO_PORTC, GPIO_PIN1, GPIO_OUTPUT);

    /* Set Timer2 period register for desired frequency */
    PR2 = PWM_PR2_VALUE;

    /* Zero the duty cycle initially */
    CCPR2L = 0;
    CLR_BIT(CCP2CON, DC2B0_BIT);
    CLR_BIT(CCP2CON, DC2B1_BIT);

    /* Set CCP2 to PWM mode (bits 3:0 = 0b1100) */
    CCP2CON = (CCP2CON & 0xF0) | CCP2_PWM_MODE;

    /* Configure T2CON: set prescaler, timer off until PWM_Start() */
    T2CON = (T2CON & 0xF8) | (PWM_T2_PRESCALER & 0x03);
    CLR_BIT(T2CON, TMR2ON_BIT);
}

void PWM_SetDutyCycle(u8 duty)
{
    u16 duty_count;

    if(duty > 100) { duty = 100; }

    duty_count = (u16)(((u16)(PR2 + 1U) * 4U * (u16)duty) / 100U);

    CCPR2L = (u8)(duty_count >> 2);

    if(GET_BIT(duty_count, 0)) { SET_BIT(CCP2CON, DC2B0_BIT); }
    else                        { CLR_BIT(CCP2CON, DC2B0_BIT); }

    if(GET_BIT(duty_count, 1)) { SET_BIT(CCP2CON, DC2B1_BIT); }
    else                        { CLR_BIT(CCP2CON, DC2B1_BIT); }
}

void PWM_Start(void)
{
    SET_BIT(T2CON, TMR2ON_BIT);
}

void PWM_Stop(void)
{
    CLR_BIT(T2CON, TMR2ON_BIT);
}
