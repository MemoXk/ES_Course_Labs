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
#include "../MCAL/TIMER_1/TIMER_1_Interface.h"

#define HB_PORT      GPIO_PORTB
#define HB_PIN       GPIO_PIN0
#define DIAG_TRIG_PORT GPIO_PORTB
#define DIAG_TRIG_PIN  GPIO_PIN1
#define DIAG_ECHO_PORT GPIO_PORTB
#define DIAG_ECHO_PIN  GPIO_PIN2
#define DRIVE_DUTY   65U     /* PWM duty cycle for motor enable */
#define US_TIMEOUT_TICKS 60000U /* Timer1 1:2 @ 20 MHz = 0.4 us/tick, 24 ms */
#define US_MIN_WIDTH_TICKS 145U /* about 1 cm; below this is a false/glitch pulse */
#define US_SAMPLE_COUNT   5U
#define US_MIN_VALID      3U
#define US_MAX_VALID_CM   150U
#define US_NO_ECHO_CM    999U

/* ---- forward decls ---- */
static void process_cmd(u8 byte);
static void uart_write_str(const char* s);
static void uart_write_u16(u16 v);
static u16 front_ultrasonic_cm(u8* status, u16* pulse_ticks);
static u16 front_filtered_cm(u8* valid_count);

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

static u16 front_ultrasonic_cm(u8* status, u16* pulse_ticks)
{
    u16 width_ticks = 0;

    *status = 'O';
    *pulse_ticks = 0;

    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN) == GPIO_HIGH)
    {
        if(TIMER1_GetValue() >= US_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'H';
            return US_NO_ECHO_CM;
        }
    }
    TIMER1_Stop();

    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);
    __delay_us(2);
    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_HIGH);
    __delay_us(10);
    GPIO_SetPinValue(DIAG_TRIG_PORT, DIAG_TRIG_PIN, GPIO_LOW);

    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN) == GPIO_LOW)
    {
        if(TIMER1_GetValue() >= US_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'N';
            return US_NO_ECHO_CM;
        }
    }

    TIMER1_Reset();
    while(GPIO_GetPinValue(DIAG_ECHO_PORT, DIAG_ECHO_PIN) == GPIO_HIGH)
    {
        width_ticks = TIMER1_GetValue();
        if(width_ticks >= US_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'T';
            *pulse_ticks = width_ticks;
            return US_NO_ECHO_CM;
        }
    }
    TIMER1_Stop();

    *pulse_ticks = width_ticks;

    if(width_ticks < US_MIN_WIDTH_TICKS)
    {
        *status = 'S';
        return US_NO_ECHO_CM;
    }

    /* Timer1 tick = 0.4 us, HC-SR04 cm = echo_us / 58, so cm = ticks / 145. */
    return (u16)(((u32)width_ticks + 72UL) / 145UL);
}

static u16 front_filtered_cm(u8* valid_count)
{
    u16 samples[US_SAMPLE_COUNT];
    u16 sample_cm;
    u16 pulse_ticks;
    u16 temp;
    u8  status;
    u8  count = 0;
    u8  i;
    u8  j;

    for(i = 0; i < US_SAMPLE_COUNT; i++)
    {
        sample_cm = front_ultrasonic_cm(&status, &pulse_ticks);

        uart_write_str("DIAG:R=");
        uart_write_u16(sample_cm);
        uart_write_str(",S=");
        UART_Write(status);
        uart_write_str(",W=");
        uart_write_u16(pulse_ticks);
        uart_write_str("\r\n");

        if(status == 'O' && sample_cm != US_NO_ECHO_CM && sample_cm <= US_MAX_VALID_CM)
        {
            samples[count] = sample_cm;
            count++;
        }

        __delay_ms(70);
    }

    *valid_count = count;
    if(count < US_MIN_VALID)
    {
        return US_NO_ECHO_CM;
    }

    for(i = 0; i < count; i++)
    {
        for(j = (u8)(i + 1U); j < count; j++)
        {
            if(samples[j] < samples[i])
            {
                temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }

    return samples[count / 2U];
}

/* =================================================================
 *  Public entry — main() calls this
 * ================================================================= */
void MANUAL_CONTROL_Test(void)
{
    u16 hb_tick = 0;
    u16 front_cm;
    u8  i;
    u8  n;
    u8  valid_count;

    /* Heartbeat LED on RB0 */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* New-hex visual signature: four quick flashes, one long flash, four quick flashes. */
    for(n = 0; n < 4U; n++)
    {
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(80);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(120);
    }
    __delay_ms(250);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
    __delay_ms(700);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
    __delay_ms(250);
    for(n = 0; n < 4U; n++)
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
    TIMER1_Init();

    /* Safe HC-SR04 timing: ECHO is input, TRIG is high for only 10 us. */
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
    uart_write_str("DIAG:FRONT_SAFE_MEDIAN_10US_RB1_RB2\r\n");

    while(1)
    {
        /* Check for command captured by ISR */
        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        front_cm = front_filtered_cm(&valid_count);
        __delay_ms(30);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        uart_write_str("US:F=");
        uart_write_u16(front_cm);
        uart_write_str("\r\n");
        uart_write_str("DIAG:MED:N=");
        uart_write_u16(valid_count);
        uart_write_str("\r\n");

        for(i = 0; i < 5U; i++)
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
