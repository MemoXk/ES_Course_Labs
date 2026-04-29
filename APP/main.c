/*
 * main.c — Application entry point
 *
 * Each driver is tested by its dedicated test file.
 * This file only calls those test functions; no register access here.
 */

#include "gpio_test.h"
#include "ext_int_test.h"
#include "timer0_test.h"
#include "pwm_test.h"
#include "adc_test.h"
#include "uart_test.h"
#include "spi_test.h"
#include "i2c_test.h"
#include "motor_test.h"

int main(void)
{
    MOTOR_Test();       /* MOTOR: forward/backward/stop with 2s delays */
    GPIO_Test();        /* GPIO: direction, write, read           */
    UART_Test();        /* UART: TX string, RX echo callback      */
    EXT_INT_Test();     /* EXT INT: edge config, callback, enable */
    TIMER0_Test();      /* Timer0: overflow callback, LED toggle  */
    ADC_Test();         /* ADC: channel read, threshold LED       */
    PWM_Test();         /* PWM: init, duty-cycle sweep            */
    SPI_Test();         /* SPI: master init, send/receive         */
    I2C_Test();         /* I2C: master init, write/read sequence  */

    while(1)
    {
        /* Interrupt-driven tasks run in the background */
    }

    return 0;
}