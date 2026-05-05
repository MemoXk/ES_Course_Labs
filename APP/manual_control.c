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

/* ---- forward decls ---- */
static void process_cmd(u8 byte);
static void uart_write_str(const char* s);
static void uart_write_u16(u16 v);
static void report_pin_state(u8 set_value);

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

static void report_pin_state(u8 set_value)
{
    uart_write_str("PIN:SET=");
    uart_write_u16(set_value);
    uart_write_str(",RB1=");
    uart_write_u16(GPIO_GetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN));
    uart_write_str(",RB2=");
    uart_write_u16(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN));
    uart_write_str("\r\n");
}

/* =================================================================
 *  Public entry — main() calls this
 * ================================================================= */
void MANUAL_CONTROL_Test(void)
{
    u16 hb_tick = 0;
    u8  i;
    u8  n;

    /* Heartbeat LED on RB0 */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* New-hex visual signature: five quick flashes, pause, five quick flashes. */
    for(n = 0; n < 5U; n++)
    {
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(80);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(120);
    }
    __delay_ms(700);
    for(n = 0; n < 5U; n++)
    {
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(80);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(120);
    }
    __delay_ms(300);

    /* Motors + PWM */
    MOTOR_Init();
    PWM_Init();
    PWM_SetDutyCycle(DRIVE_DUTY);
    PWM_Start();

    /* Trigger proof only: hold RB1 low/high and report RB1/RB2 levels. */
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
    uart_write_str("DIAG:FRONT_TRIGGER_HOLD_RB1_RB2\r\n");

    while(1)
    {
        /* Check for command captured by ISR */
        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(1000);
        report_pin_state(0);

        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_HIGH);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(1000);
        report_pin_state(1);

        GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(200);
        report_pin_state(0);

        for(i = 0; i < 2U; i++)
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
