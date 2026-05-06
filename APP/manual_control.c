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
#define FRONT_TRIG_PORT GPIO_PORTB
#define FRONT_TRIG_PIN  GPIO_PIN1
#define FRONT_ECHO_PORT GPIO_PORTB
#define FRONT_ECHO_PIN  GPIO_PIN2
#define RIGHT_TRIG_PORT GPIO_PORTB
#define RIGHT_TRIG_PIN  GPIO_PIN3
#define RIGHT_ECHO_PORT GPIO_PORTB
#define RIGHT_ECHO_PIN  GPIO_PIN4
#define LEFT_TRIG_PORT  GPIO_PORTB
#define LEFT_TRIG_PIN   GPIO_PIN5
#define LEFT_ECHO_PORT  GPIO_PORTB
#define LEFT_ECHO_PIN   GPIO_PIN6
#define DRIVE_DUTY   65U     /* PWM duty cycle for motor enable */
#define US_TIMEOUT_TICKS 60000U /* Timer1 1:2 @ 20 MHz = 0.4 us/tick, 24 ms */
#define US_MIN_WIDTH_TICKS 145U /* about 1 cm; below this is a false/glitch pulse */
#define US_SAMPLE_COUNT   5U
#define US_MIN_VALID      3U
#define US_MAX_VALID_CM   400U
#define US_INTER_PING_MS  60U
#define US_NO_ECHO_CM    999U

/* ---- forward decls ---- */
static void process_cmd(u8 byte);
static void uart_write_str(const char* s);
static void uart_write_u16(u16 v);
static u16 ultrasonic_cm(u8 trig_port, u8 trig_pin, u8 echo_port, u8 echo_pin,
                         u8* status, u16* pulse_ticks);
static void add_valid_sample(u16 samples[], u8* count, u16 sample_cm, u8 status);
static u16 median_or_no_echo(u16 samples[], u8 count);
static void ultrasonic_init_sensor(u8 trig_port, u8 trig_pin, u8 echo_port, u8 echo_pin);
static void delay_with_cmd_checks(u8 ticks_10ms);

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

static u16 ultrasonic_cm(u8 trig_port, u8 trig_pin, u8 echo_port, u8 echo_pin,
                         u8* status, u16* pulse_ticks)
{
    u16 width_ticks = 0;

    *status = 'O';
    *pulse_ticks = 0;

    TIMER1_Stop();
    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_HIGH)
    {
        if(TIMER1_GetValue() >= US_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'H';
            return US_NO_ECHO_CM;
        }
    }
    TIMER1_Stop();

    GPIO_SetPinValue(trig_port, trig_pin, GPIO_LOW);
    __delay_us(2);
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_HIGH);
    __delay_us(10);
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_LOW);

    TIMER1_Stop();
    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_LOW)
    {
        if(TIMER1_GetValue() >= US_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'N';
            return US_NO_ECHO_CM;
        }
    }

    TIMER1_Stop();
    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_HIGH)
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

static void add_valid_sample(u16 samples[], u8* count, u16 sample_cm, u8 status)
{
    if(status == 'O' && sample_cm != US_NO_ECHO_CM && sample_cm <= US_MAX_VALID_CM)
    {
        samples[*count] = sample_cm;
        (*count)++;
    }
}

static u16 median_or_no_echo(u16 samples[], u8 count)
{
    u16 temp;
    u8  i;
    u8  j;

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

static void ultrasonic_init_sensor(u8 trig_port, u8 trig_pin, u8 echo_port, u8 echo_pin)
{
    GPIO_SetPinDirection(trig_port, trig_pin, GPIO_OUTPUT);
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_LOW);
    GPIO_SetPinDirection(echo_port, echo_pin, GPIO_INPUT);
}

static void delay_with_cmd_checks(u8 ticks_10ms)
{
    u8 i;

    for(i = 0; i < ticks_10ms; i++)
    {
        __delay_ms(10);
        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }
    }
}

/* =================================================================
 *  Public entry — main() calls this
 * ================================================================= */
void MANUAL_CONTROL_Test(void)
{
    u16 hb_tick = 0;
    u16 front_samples[US_SAMPLE_COUNT];
    u16 left_samples[US_SAMPLE_COUNT];
    u16 right_samples[US_SAMPLE_COUNT];
    u16 front_cm;
    u16 left_cm;
    u16 right_cm;
    u16 sample_cm;
    u16 pulse_ticks;
    u16 front_last_ticks = 0;
    u16 left_last_ticks = 0;
    u16 right_last_ticks = 0;
    u8  i;
    u8  n;
    u8  front_valid;
    u8  left_valid;
    u8  right_valid;
    u8  status;
    u8  front_last_status = 'N';
    u8  left_last_status = 'N';
    u8  right_last_status = 'N';

    /* Heartbeat LED on RB0 */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* New-hex visual signature: eleven long flashes, one quick flash. */
    for(n = 0; n < 11U; n++)
    {
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
        __delay_ms(700);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
        __delay_ms(300);
    }
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
    __delay_ms(80);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
    __delay_ms(300);

    /* Motors + PWM */
    MOTOR_Init();
    PWM_Init();
    PWM_SetDutyCycle(DRIVE_DUTY);
    PWM_Start();
    TIMER1_Init();

    /* Front+Left+Right ultrasonic test. */
    ultrasonic_init_sensor(FRONT_TRIG_PORT, FRONT_TRIG_PIN, FRONT_ECHO_PORT, FRONT_ECHO_PIN);
    ultrasonic_init_sensor(LEFT_TRIG_PORT,  LEFT_TRIG_PIN,  LEFT_ECHO_PORT,  LEFT_ECHO_PIN);
    ultrasonic_init_sensor(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, RIGHT_ECHO_PORT, RIGHT_ECHO_PIN);

    /* UART: TX first (sets SPEN + SPBRG + BRGH),
     * then full RX init (CREN + RCIE + PEIE + GIE).
     * ISR writes UART_rx_data/UART_rx_ready; main loop reads
     * via UART_RX_IsReady() / UART_RX_GetByte().              */
    UART_TX_Init();
    UART_RX_Init();

    uart_write_str("BOOT\r\n");
    uart_write_str("DIAG:ROLLBACK_23D3FC6_FLR_RAW_11L1Q\r\n");

    while(1)
    {
        /* Check for command captured by ISR */
        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);

        front_valid = 0;
        left_valid = 0;
        right_valid = 0;

        for(i = 0; i < US_SAMPLE_COUNT; i++)
        {
            sample_cm = ultrasonic_cm(FRONT_TRIG_PORT, FRONT_TRIG_PIN, FRONT_ECHO_PORT, FRONT_ECHO_PIN, &status, &pulse_ticks);
            add_valid_sample(front_samples, &front_valid, sample_cm, status);
            front_last_status = status;
            front_last_ticks = pulse_ticks;
            delay_with_cmd_checks((u8)(US_INTER_PING_MS / 10U));

            sample_cm = ultrasonic_cm(LEFT_TRIG_PORT, LEFT_TRIG_PIN, LEFT_ECHO_PORT, LEFT_ECHO_PIN, &status, &pulse_ticks);
            add_valid_sample(left_samples, &left_valid, sample_cm, status);
            left_last_status = status;
            left_last_ticks = pulse_ticks;
            delay_with_cmd_checks((u8)(US_INTER_PING_MS / 10U));

            sample_cm = ultrasonic_cm(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, RIGHT_ECHO_PORT, RIGHT_ECHO_PIN, &status, &pulse_ticks);
            add_valid_sample(right_samples, &right_valid, sample_cm, status);
            right_last_status = status;
            right_last_ticks = pulse_ticks;
            delay_with_cmd_checks((u8)(US_INTER_PING_MS / 10U));
        }

        front_cm = median_or_no_echo(front_samples, front_valid);
        left_cm = median_or_no_echo(left_samples, left_valid);
        right_cm = median_or_no_echo(right_samples, right_valid);

        __delay_ms(30);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        uart_write_str("US:F=");
        uart_write_u16(front_cm);
        uart_write_str(",L=");
        uart_write_u16(left_cm);
        uart_write_str(",R=");
        uart_write_u16(right_cm);
        uart_write_str("\r\n");
        uart_write_str("DIAG:MED:F=");
        uart_write_u16(front_valid);
        uart_write_str(",L=");
        uart_write_u16(left_valid);
        uart_write_str(",R=");
        uart_write_u16(right_valid);
        uart_write_str("\r\n");
        uart_write_str("DIAG:RAW:F=");
        UART_Write(front_last_status);
        uart_write_str(",");
        uart_write_u16(front_last_ticks);
        uart_write_str(",L=");
        UART_Write(left_last_status);
        uart_write_str(",");
        uart_write_u16(left_last_ticks);
        uart_write_str(",R=");
        UART_Write(right_last_status);
        uart_write_str(",");
        uart_write_u16(right_last_ticks);
        uart_write_str("\r\n");

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
