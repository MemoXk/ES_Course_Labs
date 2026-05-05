/*
 * manual_control.c
 *
 * UART-driven manual motor control — ISR-driven RX.
 *
 * UART_RX_Init() enables the receiver with interrupts (RCIE+PEIE+GIE).
 * When a byte arrives the ISR fires instantly, reads RCREG, and sets a
 * volatile flag pair (UART_rx_data / UART_rx_ready) with NO function-
 * pointer call — that was the PIC16 PCLATH trap that silently broke the
 * old callback approach.  The main loop checks UART_RX_IsReady() and
 * calls UART_RX_GetByte(); this works identically for manual control
 * (human-speed commands) and autonomous mode (sensor-heavy loops).
 *
 * Protocol (9600 8N1):
 *   Pi → PIC :  'F'  forward
 *               'B'  backward
 *               'L'  turn left
 *               'R'  turn right
 *               'S'  stop
 *               (any '\n' or '\r' is silently ignored)
 *
 *   PIC → Pi :  "BOOT\r\n"        once at startup
 *               "ACK:X\r\n"        after each accepted command
 *               "HB:N\r\n"         every ~1 s, N = uptime tick counter
 */

#include "manual_control.h"
#include "../MCAL/USART/USART_Interface.h"
#include "../MCAL/GPIO/GPIO_interface.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../MCAL/PWM/PWM_Interface.h"

#define HB_PORT      GPIO_PORTB
#define HB_PIN       GPIO_PIN0
#define DIAG_TRIG_PORT GPIO_PORTB
#define DIAG_TRIG_PIN  GPIO_PIN1
#define DIAG_ECHO_PORT GPIO_PORTB
#define DIAG_ECHO_PIN  GPIO_PIN2
#define DRIVE_DUTY   65U     /* PWM duty cycle for motor enable */
#define US_TIMEOUT_TICKS 3000U /* 3000 * 10 us = 30 ms */
#define US_NO_ECHO_CM    999U

/* ---- forward decls ---- */
static void process_cmd(u8 byte);
static void uart_write_str(const char* s);
static void uart_write_u16(u16 v);
static void uart_write_pulse_us(u8 pulse_mode);
static void trigger_front_pulse(u8 pulse_mode);
static u16 front_ultrasonic_cm(u8 pulse_mode, u8* status, u16* pulse_ticks);

/* =================================================================
 *  Command handler — called from main loop only, never from ISR
 * ================================================================= */
static void process_cmd(u8 byte)
{
    char ack_letter;

    switch(byte)
    {
        case 'F': MOTOR_Forward();   ack_letter = 'F'; break;
        case 'B': MOTOR_Backward();  ack_letter = 'B'; break;
        case 'L': MOTOR_TurnLeft();  ack_letter = 'L'; break;
        case 'R': MOTOR_TurnRight(); ack_letter = 'R'; break;
        case 'S': MOTOR_Stop();      ack_letter = 'S'; break;
        default:  return;   /* ignore '\r', '\n', anything else */
    }

    uart_write_str("ACK:");
    UART_Write((u8)ack_letter);
    uart_write_str("\r\n");
}

/* ---- helper: blocking string send ---- */
static void uart_write_str(const char* s)
{
    while(*s)
    {
        UART_Write((u8)*s);
        s++;
    }
}

/* ---- helper: u16 → ASCII decimal send ---- */
static void uart_write_u16(u16 v)
{
    char buf[6];
    s8   i = 0;
    s8   j;

    if(v == 0)
    {
        UART_Write('0');
        return;
    }
    while(v > 0 && i < 5)
    {
        buf[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    for(j = (s8)(i - 1); j >= 0; j--)
    {
        UART_Write((u8)buf[j]);
    }
}

static void uart_write_pulse_us(u8 pulse_mode)
{
    switch(pulse_mode)
    {
        case 0:  uart_write_str("10");   break;
        case 1:  uart_write_str("50");   break;
        case 2:  uart_write_str("100");  break;
        default: uart_write_str("1000"); break;
    }
}

static void trigger_front_pulse(u8 pulse_mode)
{
    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);
    __delay_us(5);
    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_HIGH);

    switch(pulse_mode)
    {
        case 0:  __delay_us(10);   break;
        case 1:  __delay_us(50);   break;
        case 2:  __delay_us(100);  break;
        default: __delay_ms(1);    break;
    }

    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);
}

static u16 front_ultrasonic_cm(u8 pulse_mode, u8* status, u16* pulse_ticks)
{
    u16 idle_ticks = 0;
    u16 wait_ticks = 0;
    u16 width_ticks = 0;

    *status = 'O';
    *pulse_ticks = 0;

    while(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN) == GPIO_HIGH)
    {
        if(idle_ticks >= US_TIMEOUT_TICKS)
        {
            *status = 'H';
            return US_NO_ECHO_CM;
        }
        idle_ticks++;
        __delay_us(10);
    }

    trigger_front_pulse(pulse_mode);

    while(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN) == GPIO_LOW)
    {
        if(wait_ticks >= US_TIMEOUT_TICKS)
        {
            *status = 'N';
            return US_NO_ECHO_CM;
        }
        wait_ticks++;
        __delay_us(10);
    }

    while(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN) == GPIO_HIGH)
    {
        if(width_ticks >= US_TIMEOUT_TICKS)
        {
            *status = 'T';
            *pulse_ticks = width_ticks;
            return US_NO_ECHO_CM;
        }
        width_ticks++;
        __delay_us(10);
    }

    *pulse_ticks = width_ticks;
    return (u16)(((u32)width_ticks * 10UL) / 58UL);
}

/* =================================================================
 *  Public entry — main() calls this
 * ================================================================= */
void MANUAL_CONTROL_Test(void)
{
    u16 hb_tick = 0;
    u16 front_cm;
    u16 best_cm;
    u16 pulse_ticks;
    u8  i;
    u8  n;
    u8  pulse_mode;
    u8  status;
    u8  best_status;

    /* Heartbeat LED on RB0 */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* New-hex visual signature: four slow flashes, then two quick flashes. */
    for(n = 0; n < 4U; n++)
    {
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(250);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(250);
    }
    for(n = 0; n < 2U; n++)
    {
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(80);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(120);
    }
    __delay_ms(400);

    /* Motors + PWM */
    MOTOR_Init();
    PWM_Init();
    PWM_SetDutyCycle(DRIVE_DUTY);
    PWM_Start();

    /* Front ultrasonic pulse diagnostic only. */
    GPIO_SetPinDirection(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);
    GPIO_SetPinDirection(DIAG_ECHO_PORT, DIAG_ECHO_PIN, GPIO_INPUT);

    /* UART: TX first (sets SPEN + SPBRG + BRGH),
     * then full RX init (CREN + RCIE + PEIE + GIE).
     * ISR writes UART_rx_data/UART_rx_ready; main loop reads
     * via UART_RX_IsReady() / UART_RX_GetByte().              */
    UART_TX_Init();
    UART_RX_Init();

    uart_write_str("BOOT\r\n");
    uart_write_str("DIAG:FRONT_PULSE_LADDER_RB1_RB2\r\n");

    while(1)
    {
        /* Check for command captured by ISR */
        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        best_cm = US_NO_ECHO_CM;
        best_status = 'N';

        for(pulse_mode = 0; pulse_mode < 4U; pulse_mode++)
        {
            front_cm = front_ultrasonic_cm(pulse_mode, &status, &pulse_ticks);

            uart_write_str("DIAG:P=");
            uart_write_pulse_us(pulse_mode);
            uart_write_str(",S=");
            UART_Write(status);
            uart_write_str(",W=");
            uart_write_u16(pulse_ticks);
            uart_write_str("\r\n");

            if(status == 'O' && best_cm == US_NO_ECHO_CM)
            {
                best_cm = front_cm;
                best_status = status;
            }

            __delay_ms(70);
        }

        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        uart_write_str("US:F=");
        uart_write_u16(best_cm);
        uart_write_str("\r\n");
        uart_write_str("DIAG:BEST=");
        UART_Write(best_status);
        uart_write_str("\r\n");

        for(i = 0; i < 6U; i++)
        {
            __delay_ms(100);
            if(UART_RX_IsReady())
            {
                process_cmd(UART_RX_GetByte());
            }
        }

        hb_tick++;
        uart_write_str("HB:");
        uart_write_u16(hb_tick);
        uart_write_str("\r\n");
    }
}
