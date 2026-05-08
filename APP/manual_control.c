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
 *               "WARN:...\r\n"      when obstacle guard blocks/stops motion
 *               "US:...\r\n"        ultrasonic telemetry for the Pi UI
 *               "LDR:...\r\n"       light-state telemetry for the Pi UI
 *               "BELT:...\r\n"      seat-belt switch telemetry for the Pi UI
 *               "HB:N\r\n"         every ~1 s, N = uptime tick counter
 */

#include "manual_control.h"
#include "../MCAL/USART/USART_Interface.h"
#include "../MCAL/GPIO/GPIO_interface.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../MCAL/PWM/PWM_Interface.h"
#include "../MCAL/TIMER_1/TIMER_1_Interface.h"

#define FRONT_TRIG_PORT GPIO_PORTB
#define FRONT_TRIG_PIN  GPIO_PIN1
#define FRONT_ECHO_PORT GPIO_PORTB
#define FRONT_ECHO_PIN  GPIO_PIN2
#define LEFT_TRIG_PORT  GPIO_PORTB
#define LEFT_TRIG_PIN   GPIO_PIN3
#define LEFT_ECHO_PORT  GPIO_PORTB
#define LEFT_ECHO_PIN   GPIO_PIN4
#define RIGHT_TRIG_PORT GPIO_PORTB
#define RIGHT_TRIG_PIN  GPIO_PIN5
#define RIGHT_ECHO_PORT GPIO_PORTB
#define RIGHT_ECHO_PIN  GPIO_PIN6
#define SEATBELT_PORT   GPIO_PORTB
#define SEATBELT_PIN    GPIO_PIN0
#define SEATBELT_ON_LEVEL GPIO_HIGH
#define LDR_DO_PORT     GPIO_PORTD
#define LDR_DO_PIN      GPIO_PIN4
#define LDR_LED_PORT    GPIO_PORTD
#define LDR_LED_PIN     GPIO_PIN5
#define LDR_DARK_LEVEL  GPIO_HIGH
#define DRIVE_DUTY   65U     /* PWM duty cycle for motor enable */
#define US_TIMEOUT_TICKS 60000U /* Timer1 1:2 @ 20 MHz = 0.4 us/tick, 24 ms */
#define US_MIN_WIDTH_TICKS 145U /* about 1 cm; below this is a false/glitch pulse */
#define US_SAMPLE_COUNT   3U
#define US_MIN_VALID      2U
#define US_MAX_VALID_CM   400U
#define US_INTER_PING_MS  60U
#define US_NO_ECHO_CM    999U
#define OBSTACLE_BLOCK_CM 25U

/* ---- forward decls ---- */
static void process_cmd(u8 byte);
static void uart_write_str(const char* s);
static void uart_write_u16(u16 v);
static u8 command_requires_seatbelt(u8 cmd);
static u8 command_block_distance(u8 cmd, u16* cm);
static void enforce_seatbelt_stop(void);
static void enforce_active_obstacle_stop(void);
static u16 ultrasonic_cm(u8 trig_port, u8 trig_pin, u8 echo_port, u8 echo_pin,
                         u8* status, u16* pulse_ticks);
static void add_valid_sample(u16 samples[], u8* count, u16 sample_cm, u8 status);
static u16 median_or_no_echo(u16 samples[], u8 count);
static void ultrasonic_init_sensor(u8 trig_port, u8 trig_pin, u8 echo_port, u8 echo_pin);

static u16 latest_front_cm = US_NO_ECHO_CM;
static u16 latest_left_cm = US_NO_ECHO_CM;
static u16 latest_right_cm = US_NO_ECHO_CM;
static u8  latest_ldr_raw = GPIO_HIGH;
static u8  latest_ldr_dark = 0U;
static u8  latest_seatbelt_raw = GPIO_LOW;
static u8  latest_seatbelt_on = 0U;
static u8  active_drive_cmd = 'S';

#define CHECK_RX_CMD()                  \
    do                                  \
    {                                   \
        SEATBELT_UPDATE();              \
        enforce_seatbelt_stop();        \
        if(UART_RX_IsReady())           \
        {                               \
            process_cmd(UART_RX_GetByte()); \
        }                               \
    } while(0)

#define DELAY_WITH_CMD_CHECKS(ticks_10ms)                 \
    do                                                    \
    {                                                     \
        u8 delay_i;                                       \
        for(delay_i = 0; delay_i < (ticks_10ms); delay_i++) \
        {                                                 \
            __delay_ms(10);                               \
            CHECK_RX_CMD();                               \
        }                                                 \
    } while(0)

#define UART_WRITE_OBSTACLE_WARN(reason, cmd, cm) \
    do                                           \
    {                                            \
        uart_write_str("WARN:");                 \
        uart_write_str(reason);                  \
        uart_write_str(":");                     \
        UART_Write(cmd);                         \
        uart_write_str("=");                     \
        uart_write_u16(cm);                      \
        uart_write_str(",T=");                   \
        uart_write_u16(OBSTACLE_BLOCK_CM);       \
        uart_write_str("\r\n");                 \
    } while(0)

#define UART_WRITE_SEATBELT_WARN()      \
    do                                  \
    {                                   \
        uart_write_str("WARN:BELT:");   \
        uart_write_u16(latest_seatbelt_on); \
        uart_write_str("\r\n");         \
    } while(0)

#define SEATBELT_UPDATE()                                           \
    do                                                              \
    {                                                               \
        latest_seatbelt_raw = GPIO_GetPinValue(SEATBELT_PORT, SEATBELT_PIN); \
        latest_seatbelt_on = (u8)(latest_seatbelt_raw == SEATBELT_ON_LEVEL); \
    } while(0)

#define LDR_UPDATE_OUTPUTS()                                      \
    do                                                            \
    {                                                             \
        latest_ldr_raw = GPIO_GetPinValue(LDR_DO_PORT, LDR_DO_PIN); \
        latest_ldr_dark = (u8)(latest_ldr_raw == LDR_DARK_LEVEL); \
        GPIO_SetPinValue(LDR_LED_PORT, LDR_LED_PIN,               \
                         latest_ldr_dark ? GPIO_HIGH : GPIO_LOW); \
    } while(0)

/* =================================================================
 *  Command handler — called from main loop only, never from ISR
 * ================================================================= */
static void process_cmd(u8 byte)
{
    char ack_letter;
    u16 blocked_cm = US_NO_ECHO_CM;

    if(byte == 'S')
    {
        MOTOR_Stop();
        active_drive_cmd = 'S';
        ack_letter = 'S';
    }
    else if(command_requires_seatbelt(byte) && !latest_seatbelt_on)
    {
        MOTOR_Stop();
        active_drive_cmd = 'S';
        UART_WRITE_SEATBELT_WARN();
        return;
    }
    else if(command_block_distance(byte, &blocked_cm))
    {
        MOTOR_Stop();
        active_drive_cmd = 'S';
        UART_WRITE_OBSTACLE_WARN("BLOCK", byte, blocked_cm);
        return;
    }
    else
    {
        switch(byte)
        {
            case 'F': MOTOR_Forward();   ack_letter = 'F'; break;
            case 'B': MOTOR_Backward();  ack_letter = 'B'; break;
            case 'L': MOTOR_TurnLeft();  ack_letter = 'L'; break;
            case 'R': MOTOR_TurnRight(); ack_letter = 'R'; break;
            default:  return;   /* ignore '\r', '\n', anything else */
        }
        active_drive_cmd = (u8)ack_letter;
    }

    uart_write_str("ACK:");
    UART_Write((u8)ack_letter);
    uart_write_str("\r\n");
}

static u8 command_requires_seatbelt(u8 cmd)
{
    return (u8)(cmd == 'F' || cmd == 'B' || cmd == 'L' || cmd == 'R');
}

static u8 command_block_distance(u8 cmd, u16* cm)
{
    switch(cmd)
    {
        case 'F':
            *cm = latest_front_cm;
            break;
        case 'L':
            *cm = latest_left_cm;
            break;
        case 'R':
            *cm = latest_right_cm;
            break;
        default:
            *cm = US_NO_ECHO_CM;
            return 0U;
    }

    return (u8)((*cm != US_NO_ECHO_CM) && (*cm <= OBSTACLE_BLOCK_CM));
}

static void enforce_seatbelt_stop(void)
{
    if(!latest_seatbelt_on && active_drive_cmd != 'S')
    {
        MOTOR_Stop();
        active_drive_cmd = 'S';
        UART_WRITE_SEATBELT_WARN();
    }
}

static void enforce_active_obstacle_stop(void)
{
    u16 blocked_cm = US_NO_ECHO_CM;

    if(command_block_distance(active_drive_cmd, &blocked_cm))
    {
        MOTOR_Stop();
        UART_WRITE_OBSTACLE_WARN("STOP", active_drive_cmd, blocked_cm);
        active_drive_cmd = 'S';
    }
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
    u8  i;
    u8  front_valid;
    u8  left_valid;
    u8  right_valid;
    u8  status;

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
    GPIO_SetPinDirection(SEATBELT_PORT, SEATBELT_PIN, GPIO_INPUT);
    SEATBELT_UPDATE();
    GPIO_SetPinDirection(LDR_DO_PORT, LDR_DO_PIN, GPIO_INPUT);
    GPIO_SetPinDirection(LDR_LED_PORT, LDR_LED_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(LDR_LED_PORT, LDR_LED_PIN, GPIO_LOW);
    LDR_UPDATE_OUTPUTS();

    /* UART: TX first (sets SPEN + SPBRG + BRGH),
     * then full RX init (CREN + RCIE + PEIE + GIE).
     * ISR writes UART_rx_data/UART_rx_ready; main loop reads
     * via UART_RX_IsReady() / UART_RX_GetByte().              */
    UART_TX_Init();
    UART_RX_Init();

    uart_write_str("BOOT\r\n");
    uart_write_str("DIAG:BUILD_SEATBELT_RB0_LDR_20260509_A\r\n");

    while(1)
    {
        /* Check for command captured by ISR */
        CHECK_RX_CMD();

        front_valid = 0;
        left_valid = 0;
        right_valid = 0;

        for(i = 0; i < US_SAMPLE_COUNT; i++)
        {
            sample_cm = ultrasonic_cm(FRONT_TRIG_PORT, FRONT_TRIG_PIN, FRONT_ECHO_PORT, FRONT_ECHO_PIN, &status, &pulse_ticks);
            add_valid_sample(front_samples, &front_valid, sample_cm, status);
            DELAY_WITH_CMD_CHECKS((u8)(US_INTER_PING_MS / 10U));

            sample_cm = ultrasonic_cm(LEFT_TRIG_PORT, LEFT_TRIG_PIN, LEFT_ECHO_PORT, LEFT_ECHO_PIN, &status, &pulse_ticks);
            add_valid_sample(left_samples, &left_valid, sample_cm, status);
            DELAY_WITH_CMD_CHECKS((u8)(US_INTER_PING_MS / 10U));

            sample_cm = ultrasonic_cm(RIGHT_TRIG_PORT, RIGHT_TRIG_PIN, RIGHT_ECHO_PORT, RIGHT_ECHO_PIN, &status, &pulse_ticks);
            add_valid_sample(right_samples, &right_valid, sample_cm, status);
            DELAY_WITH_CMD_CHECKS((u8)(US_INTER_PING_MS / 10U));
        }

        front_cm = median_or_no_echo(front_samples, front_valid);
        left_cm = median_or_no_echo(left_samples, left_valid);
        right_cm = median_or_no_echo(right_samples, right_valid);
        latest_front_cm = front_cm;
        latest_left_cm = left_cm;
        latest_right_cm = right_cm;
        enforce_active_obstacle_stop();
        LDR_UPDATE_OUTPUTS();
        SEATBELT_UPDATE();
        enforce_seatbelt_stop();

        CHECK_RX_CMD();

        uart_write_str("US:F=");
        uart_write_u16(front_cm);
        uart_write_str(",L=");
        uart_write_u16(left_cm);
        uart_write_str(",R=");
        uart_write_u16(right_cm);
        uart_write_str("\r\n");
        uart_write_str("LDR:D=");
        uart_write_u16(latest_ldr_dark);
        uart_write_str(",DO=");
        uart_write_u16(latest_ldr_raw);
        uart_write_str("\r\n");
        uart_write_str("BELT:S=");
        uart_write_u16(latest_seatbelt_on);
        uart_write_str(",IN=");
        uart_write_u16(latest_seatbelt_raw);
        uart_write_str("\r\n");

        for(i = 0; i < 2U; i++)
        {
            __delay_ms(100);
            CHECK_RX_CMD();
        }

        hb_tick++;
        uart_write_str("HB:");
        uart_write_u16(hb_tick);
        uart_write_str("\r\n");
    }
}
