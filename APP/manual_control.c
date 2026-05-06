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
#define SHARED_TRIG_PORT GPIO_PORTB
#define SHARED_TRIG_PIN  GPIO_PIN1
#define FRONT_ECHO_PORT GPIO_PORTB
#define FRONT_ECHO_PIN  GPIO_PIN2
#define LEFT_ECHO_PORT  GPIO_PORTB
#define LEFT_ECHO_PIN   GPIO_PIN3
#define BACK_ECHO_PORT  GPIO_PORTB
#define BACK_ECHO_PIN   GPIO_PIN4
#define DRIVE_DUTY   65U     /* PWM duty cycle for motor enable */
#define US_TIMEOUT_TICKS 60000U /* Timer1 1:2 @ 20 MHz = 0.4 us/tick, 24 ms */
#define US_MIN_WIDTH_TICKS 145U /* about 1 cm; below this is a false/glitch pulse */
#define US_SAMPLE_COUNT   5U
#define US_MIN_VALID      3U
#define US_MAX_VALID_CM   400U
#define US_INTER_PING_MS  60U
#define US_NO_ECHO_CM    999U
#define US_FRONT_IDX      0U
#define US_BACK_IDX       1U
#define US_LEFT_IDX       2U
#define US_SENSOR_COUNT   3U

/* ---- forward decls ---- */
static void process_cmd(u8 byte);
static void uart_write_str(const char* s);
static void uart_write_u16(u16 v);
static void ultrasonic_shared_sample(u16 cm[], u8 status[], u16 pulse_ticks[]);
static void add_valid_sample(u16 samples[], u8* count, u16 sample_cm, u8 status);
static u16 median_or_no_echo(u16 samples[], u8 count);
static void ultrasonic_init_shared(void);
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

static void ultrasonic_finish_sample(u16 cm[], u8 status[], u16 pulse_ticks[],
                                     u8 index, u8 seen, u8 done, u16 width_ticks)
{
    cm[index] = US_NO_ECHO_CM;
    pulse_ticks[index] = width_ticks;

    if(seen == 0U)
    {
        status[index] = 'N';
    }
    else if(done == 0U)
    {
        status[index] = 'T';
    }
    else if(width_ticks < US_MIN_WIDTH_TICKS)
    {
        status[index] = 'S';
    }
    else
    {
        status[index] = 'O';
        cm[index] = (u16)(((u32)width_ticks + 72UL) / 145UL);
    }
}

static void ultrasonic_shared_sample(u16 cm[], u8 status[], u16 pulse_ticks[])
{
    u16 now;
    u16 front_start = 0;
    u16 back_start = 0;
    u16 left_start = 0;
    u16 front_width = 0;
    u16 back_width = 0;
    u16 left_width = 0;
    u8  front_seen = 0;
    u8  back_seen = 0;
    u8  left_seen = 0;
    u8  front_done = 0;
    u8  back_done = 0;
    u8  left_done = 0;
    u8  front_level;
    u8  back_level;
    u8  left_level;

    cm[US_FRONT_IDX] = US_NO_ECHO_CM;
    cm[US_BACK_IDX] = US_NO_ECHO_CM;
    cm[US_LEFT_IDX] = US_NO_ECHO_CM;
    status[US_FRONT_IDX] = 'N';
    status[US_BACK_IDX] = 'N';
    status[US_LEFT_IDX] = 'N';
    pulse_ticks[US_FRONT_IDX] = 0;
    pulse_ticks[US_BACK_IDX] = 0;
    pulse_ticks[US_LEFT_IDX] = 0;

    TIMER1_Reset();
    TIMER1_Start();
    while((GPIO_GetPinValue(FRONT_ECHO_PORT, FRONT_ECHO_PIN) == GPIO_HIGH) ||
          (GPIO_GetPinValue(BACK_ECHO_PORT, BACK_ECHO_PIN) == GPIO_HIGH) ||
          (GPIO_GetPinValue(LEFT_ECHO_PORT, LEFT_ECHO_PIN) == GPIO_HIGH))
    {
        if(TIMER1_GetValue() >= US_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            status[US_FRONT_IDX] = 'H';
            status[US_BACK_IDX] = 'H';
            status[US_LEFT_IDX] = 'H';
            return;
        }
    }
    TIMER1_Stop();

    GPIO_SetPinValue(SHARED_TRIG_PORT, SHARED_TRIG_PIN, GPIO_LOW);
    __delay_us(2);
    GPIO_SetPinValue(SHARED_TRIG_PORT, SHARED_TRIG_PIN, GPIO_HIGH);
    __delay_us(10);
    GPIO_SetPinValue(SHARED_TRIG_PORT, SHARED_TRIG_PIN, GPIO_LOW);

    TIMER1_Reset();
    TIMER1_Start();
    while(TIMER1_GetValue() < US_TIMEOUT_TICKS &&
          ((front_done == 0U) || (back_done == 0U) || (left_done == 0U)))
    {
        now = TIMER1_GetValue();
        front_level = GPIO_GetPinValue(FRONT_ECHO_PORT, FRONT_ECHO_PIN);
        back_level = GPIO_GetPinValue(BACK_ECHO_PORT, BACK_ECHO_PIN);
        left_level = GPIO_GetPinValue(LEFT_ECHO_PORT, LEFT_ECHO_PIN);

        if(front_done == 0U)
        {
            if(front_seen == 0U)
            {
                if(front_level == GPIO_HIGH)
                {
                    front_seen = 1U;
                    front_start = now;
                }
            }
            else if(front_level == GPIO_LOW)
            {
                front_width = (u16)(now - front_start);
                front_done = 1U;
            }
        }

        if(back_done == 0U)
        {
            if(back_seen == 0U)
            {
                if(back_level == GPIO_HIGH)
                {
                    back_seen = 1U;
                    back_start = now;
                }
            }
            else if(back_level == GPIO_LOW)
            {
                back_width = (u16)(now - back_start);
                back_done = 1U;
            }
        }

        if(left_done == 0U)
        {
            if(left_seen == 0U)
            {
                if(left_level == GPIO_HIGH)
                {
                    left_seen = 1U;
                    left_start = now;
                }
            }
            else if(left_level == GPIO_LOW)
            {
                left_width = (u16)(now - left_start);
                left_done = 1U;
            }
        }
    }
    TIMER1_Stop();

    ultrasonic_finish_sample(cm, status, pulse_ticks, US_FRONT_IDX, front_seen, front_done, front_width);
    ultrasonic_finish_sample(cm, status, pulse_ticks, US_BACK_IDX, back_seen, back_done, back_width);
    ultrasonic_finish_sample(cm, status, pulse_ticks, US_LEFT_IDX, left_seen, left_done, left_width);
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

static void ultrasonic_init_shared(void)
{
    GPIO_SetPinDirection(SHARED_TRIG_PORT, SHARED_TRIG_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(SHARED_TRIG_PORT, SHARED_TRIG_PIN, GPIO_LOW);

    GPIO_SetPinDirection(FRONT_ECHO_PORT, FRONT_ECHO_PIN, GPIO_INPUT);
    GPIO_SetPinDirection(BACK_ECHO_PORT, BACK_ECHO_PIN, GPIO_INPUT);
    GPIO_SetPinDirection(LEFT_ECHO_PORT, LEFT_ECHO_PIN, GPIO_INPUT);

    GPIO_SetPinDirection(GPIO_PORTB, GPIO_PIN5, GPIO_INPUT);
    GPIO_SetPinDirection(GPIO_PORTB, GPIO_PIN6, GPIO_INPUT);
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
    u16 back_samples[US_SAMPLE_COUNT];
    u16 left_samples[US_SAMPLE_COUNT];
    u16 front_cm;
    u16 back_cm;
    u16 left_cm;
    u16 sample_cm[US_SENSOR_COUNT];
    u16 pulse_ticks[US_SENSOR_COUNT];
    u8  i;
    u8  n;
    u8  front_valid;
    u8  back_valid;
    u8  left_valid;
    u8  status[US_SENSOR_COUNT];

    /* Heartbeat LED on RB0 */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* New-hex visual signature: one long flash, seven quick flashes. */
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
    __delay_ms(900);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);
    __delay_ms(300);
    for(n = 0; n < 7U; n++)
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

    /* Safe HC-SR04 timing: ECHO pins are inputs, shared TRIG is high for only 10 us. */
    ultrasonic_init_shared();

    /* UART: TX first (sets SPEN + SPBRG + BRGH),
     * then full RX init (CREN + RCIE + PEIE + GIE).
     * ISR writes UART_rx_data/UART_rx_ready; main loop reads
     * via UART_RX_IsReady() / UART_RX_GetByte().              */
    UART_TX_Init();
    UART_RX_Init();

    uart_write_str("BOOT\r\n");
    uart_write_str("DIAG:US3_SHARED_TRIG_RB1_ECHO_F2_B4_L3\r\n");

    while(1)
    {
        /* Check for command captured by ISR */
        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);

        front_valid = 0;
        back_valid = 0;
        left_valid = 0;

        for(i = 0; i < US_SAMPLE_COUNT; i++)
        {
            ultrasonic_shared_sample(sample_cm, status, pulse_ticks);
            add_valid_sample(front_samples, &front_valid, sample_cm[US_FRONT_IDX], status[US_FRONT_IDX]);
            add_valid_sample(back_samples, &back_valid, sample_cm[US_BACK_IDX], status[US_BACK_IDX]);
            add_valid_sample(left_samples, &left_valid, sample_cm[US_LEFT_IDX], status[US_LEFT_IDX]);
            delay_with_cmd_checks((u8)(US_INTER_PING_MS / 10U));
        }

        front_cm = median_or_no_echo(front_samples, front_valid);
        back_cm = median_or_no_echo(back_samples, back_valid);
        left_cm = median_or_no_echo(left_samples, left_valid);

        __delay_ms(30);
        GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

        if(UART_RX_IsReady())
        {
            process_cmd(UART_RX_GetByte());
        }

        uart_write_str("US:F=");
        uart_write_u16(front_cm);
        uart_write_str(",B=");
        uart_write_u16(back_cm);
        uart_write_str(",L=");
        uart_write_u16(left_cm);
        uart_write_str("\r\n");
        uart_write_str("DIAG:MED:F=");
        uart_write_u16(front_valid);
        uart_write_str(",B=");
        uart_write_u16(back_valid);
        uart_write_str(",L=");
        uart_write_u16(left_valid);
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
