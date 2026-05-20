/*
 * manual_control.c
 *
 * Application policy for the UART-driven car.
 *
 * The low-level details live below this file:
 *   - UART RX ring buffer: MCAL/USART
 *   - GPIO register access: MCAL/GPIO
 *   - motor direction pins: HAL/MOTOR
 *   - ultrasonic timing/filtering: HAL/ULTRASONIC over MCAL/TIMER_1
 *   - seat-belt/LDR digital inputs: HAL/SWITCH
 *   - LDR indicator LED: HAL/LED
 *   - CCP1 PWM output: MCAL/PWM
 *
 * Protocol, 9600 8N1:
 *   Pi -> PIC : F/B/L/R/S movement, U/D throttle
 *   PIC -> Pi : BOOT, DIAG, ACK, WARN, US, LDR, BELT, SPD, HB
 */

#include "manual_control.h"
#include "manual_control_config.h"
#include "../MCAL/USART/USART_Interface.h"
#include "../MCAL/PWM/PWM_Interface.h"
#include "../HAL/MOTOR/MOTOR_interface.h"
#include "../HAL/ULTRASONIC/ULTRASONIC_interface.h"
#include "../HAL/SWITCH/Switch_interface.h"
#include "../HAL/LED/LED_interface.h"

#define SPEED_STEP_OK     0U
#define SPEED_STEP_NEAR   1U
#define SPEED_STEP_LIMIT  2U

static void process_cmd(u8 byte);

static void seatbelt_init(void);
static void seatbelt_update(void);
static void enforce_seatbelt_stop(void);

static void ldr_init(void);
static void ldr_update_outputs(void);

static void drive_speed_init(void);
static void drive_speed_reset(void);
static u8   drive_speed_increase(u8* requested_duty);
static void drive_speed_decrease(void);

static void ultrasonic_init_active_sensors(void);
static void ultrasonic_sample_active_sensors(void);
static u8   command_block_distance(u8 cmd, u16* cm);
static u8   command_requires_seatbelt(u8 cmd);
static void enforce_active_obstacle_stop(void);

static void protocol_write_str(const char* s);
static void protocol_write_u16(u16 v);

static u16 latest_front_cm = ULTRASONIC_NO_OBJ;
static u16 latest_left_cm = ULTRASONIC_NO_OBJ;
static u16 latest_right_cm = ULTRASONIC_NO_OBJ;
static u8  latest_ldr_raw = GPIO_LOW;
static u8  latest_ldr_dark = 0U;
static u8  latest_seatbelt_raw = GPIO_LOW;
static u8  latest_seatbelt_on = 0U;
static u8  active_drive_cmd = 'S';
static u8  current_drive_duty = MANUAL_DRIVE_START_DUTY;

#define DRAIN_RX_COMMANDS()                                      \
    do                                                           \
    {                                                            \
        u8 rx_drain_count = 0U;                                  \
        seatbelt_update();                                       \
        enforce_seatbelt_stop();                                 \
        while(UART_RX_IsReady() && rx_drain_count < MANUAL_RX_DRAIN_LIMIT) \
        {                                                        \
            process_cmd(UART_RX_GetByte());                      \
            rx_drain_count++;                                    \
            seatbelt_update();                                   \
            enforce_seatbelt_stop();                             \
        }                                                        \
    } while(0)

#define DELAY_WITH_COMMAND_CHECKS(ticks_10ms)                    \
    do                                                           \
    {                                                            \
        u8 delay_i;                                              \
        for(delay_i = 0U; delay_i < (ticks_10ms); delay_i++)     \
        {                                                        \
            __delay_ms(MANUAL_DELAY_SLICE_MS);                   \
            DRAIN_RX_COMMANDS();                                 \
        }                                                        \
    } while(0)

#define PROTOCOL_WRITE_BOOT()                                    \
    do                                                           \
    {                                                            \
        protocol_write_str("BOOT\r\n");                         \
        protocol_write_str(MANUAL_BUILD_ID);                     \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_ACK(ack)                                  \
    do                                                           \
    {                                                            \
        protocol_write_str("ACK:");                              \
        UART_Write((u8)(ack));                                   \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_OBSTACLE_WARN(reason, cmd, cm)            \
    do                                                           \
    {                                                            \
        protocol_write_str("WARN:");                             \
        protocol_write_str((reason));                            \
        protocol_write_str(":");                                 \
        UART_Write((u8)(cmd));                                   \
        protocol_write_str("=");                                 \
        protocol_write_u16((cm));                                \
        protocol_write_str(",T=");                               \
        protocol_write_u16(ULTRASONIC_OBSTACLE_BLOCK_CM);        \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_SEATBELT_WARN()                           \
    do                                                           \
    {                                                            \
        protocol_write_str("WARN:BELT:");                        \
        protocol_write_u16(latest_seatbelt_on);                  \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_SPEED_NEAR(current, threshold)            \
    do                                                           \
    {                                                            \
        protocol_write_str("WARN:SPD:NEAR,P=");                  \
        protocol_write_u16((current));                           \
        protocol_write_str(",T=");                               \
        protocol_write_u16((threshold));                         \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_SPEED_LIMIT(requested, threshold, reset_to) \
    do                                                           \
    {                                                            \
        protocol_write_str("WARN:SPD:LIMIT,P=");                 \
        protocol_write_u16((requested));                         \
        protocol_write_str(",T=");                               \
        protocol_write_u16((threshold));                         \
        protocol_write_str(",R=");                               \
        protocol_write_u16((reset_to));                          \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_SPEED_TELEMETRY()                         \
    do                                                           \
    {                                                            \
        protocol_write_str("SPD:P=");                            \
        protocol_write_u16(current_drive_duty);                  \
        protocol_write_str(",S=");                               \
        protocol_write_u16(MANUAL_DRIVE_START_DUTY);             \
        protocol_write_str(",N=");                               \
        protocol_write_u16(MANUAL_DRIVE_MIN_DUTY);               \
        protocol_write_str(",W=");                               \
        protocol_write_u16(MANUAL_DRIVE_WARN_DUTY);              \
        protocol_write_str(",M=");                               \
        protocol_write_u16(MANUAL_DRIVE_MAX_DUTY);               \
        protocol_write_str(",K=");                               \
        protocol_write_u16(MANUAL_DRIVE_STEP_DUTY);              \
        protocol_write_str("\r\n");                             \
    } while(0)

#define PROTOCOL_WRITE_SENSOR_TELEMETRY()                        \
    do                                                           \
    {                                                            \
        protocol_write_str("US:F=");                             \
        protocol_write_u16(latest_front_cm);                     \
        protocol_write_str(",L=");                               \
        protocol_write_u16(latest_left_cm);                      \
        protocol_write_str(",R=");                               \
        protocol_write_u16(latest_right_cm);                     \
        protocol_write_str("\r\n");                             \
        DRAIN_RX_COMMANDS();                                     \
        protocol_write_str("LDR:D=");                            \
        protocol_write_u16(latest_ldr_dark);                     \
        protocol_write_str(",DO=");                              \
        protocol_write_u16(latest_ldr_raw);                      \
        protocol_write_str("\r\n");                             \
        DRAIN_RX_COMMANDS();                                     \
        protocol_write_str("BELT:S=");                           \
        protocol_write_u16(latest_seatbelt_on);                  \
        protocol_write_str(",IN=");                              \
        protocol_write_u16(latest_seatbelt_raw);                 \
        protocol_write_str("\r\n");                             \
        DRAIN_RX_COMMANDS();                                     \
        PROTOCOL_WRITE_SPEED_TELEMETRY();                        \
    } while(0)

static void process_cmd(u8 byte)
{
    char ack_letter;
    u16 blocked_cm = ULTRASONIC_NO_OBJ;
    u8 requested_duty;
    u8 speed_status;

    if(byte == 'U')
    {
        speed_status = drive_speed_increase(&requested_duty);
        if(speed_status == SPEED_STEP_LIMIT)
        {
            MOTOR_Stop();
            active_drive_cmd = 'S';
            PROTOCOL_WRITE_SPEED_LIMIT(requested_duty,
                                       MANUAL_DRIVE_MAX_DUTY,
                                       MANUAL_DRIVE_START_DUTY);
            PROTOCOL_WRITE_SPEED_TELEMETRY();
            return;
        }

        PROTOCOL_WRITE_ACK('U');
        if(speed_status == SPEED_STEP_NEAR)
        {
            PROTOCOL_WRITE_SPEED_NEAR(current_drive_duty, MANUAL_DRIVE_MAX_DUTY);
        }
        PROTOCOL_WRITE_SPEED_TELEMETRY();
        return;
    }

    if(byte == 'D')
    {
        drive_speed_decrease();
        PROTOCOL_WRITE_ACK('D');
        PROTOCOL_WRITE_SPEED_TELEMETRY();
        return;
    }

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
        PROTOCOL_WRITE_SEATBELT_WARN();
        return;
    }
    else if(command_block_distance(byte, &blocked_cm))
    {
        MOTOR_Stop();
        active_drive_cmd = 'S';
        PROTOCOL_WRITE_OBSTACLE_WARN("BLOCK", byte, blocked_cm);
        return;
    }
    else
    {
        switch(byte)
        {
            case 'F':
                MOTOR_Forward();
                ack_letter = 'F';
                break;
            case 'B':
                MOTOR_Backward();
                ack_letter = 'B';
                break;
            case 'L':
                MOTOR_TurnLeft();
                ack_letter = 'L';
                break;
            case 'R':
                MOTOR_TurnRight();
                ack_letter = 'R';
                break;
            default:
                return;
        }
        active_drive_cmd = (u8)ack_letter;
    }

    PROTOCOL_WRITE_ACK((u8)ack_letter);
}

static void seatbelt_init(void)
{
    Switch_Init(MANUAL_SEATBELT_PORT, MANUAL_SEATBELT_PIN, MANUAL_SEATBELT_ACTIVE);
    seatbelt_update();
}

static void seatbelt_update(void)
{
    latest_seatbelt_raw = Switch_ReadRaw(MANUAL_SEATBELT_PORT, MANUAL_SEATBELT_PIN);
    latest_seatbelt_on = (u8)(Switch_GetState(MANUAL_SEATBELT_PORT,
                                              MANUAL_SEATBELT_PIN,
                                              MANUAL_SEATBELT_ACTIVE) == SWITCH_PRESSED);
}

static void enforce_seatbelt_stop(void)
{
    if(!latest_seatbelt_on && active_drive_cmd != 'S')
    {
        MOTOR_Stop();
        active_drive_cmd = 'S';
        PROTOCOL_WRITE_SEATBELT_WARN();
    }
}

static void ldr_init(void)
{
    Switch_Init(MANUAL_LDR_DO_PORT, MANUAL_LDR_DO_PIN, MANUAL_LDR_ACTIVE);
    LED_Init(MANUAL_LDR_LED_PORT, MANUAL_LDR_LED_PIN);
    LED_Off(MANUAL_LDR_LED_PORT, MANUAL_LDR_LED_PIN);
    ldr_update_outputs();
}

static void ldr_update_outputs(void)
{
    latest_ldr_raw = Switch_ReadRaw(MANUAL_LDR_DO_PORT, MANUAL_LDR_DO_PIN);
    latest_ldr_dark = (u8)(Switch_GetState(MANUAL_LDR_DO_PORT,
                                           MANUAL_LDR_DO_PIN,
                                           MANUAL_LDR_ACTIVE) == SWITCH_PRESSED);

    if(latest_ldr_dark)
    {
        LED_On(MANUAL_LDR_LED_PORT, MANUAL_LDR_LED_PIN);
    }
    else
    {
        LED_Off(MANUAL_LDR_LED_PORT, MANUAL_LDR_LED_PIN);
    }
}

static void drive_speed_init(void)
{
    PWM_Init();
    current_drive_duty = MANUAL_DRIVE_START_DUTY;
    PWM_SetDutyCycle(current_drive_duty);
    PWM_Start();
}

static void drive_speed_reset(void)
{
    current_drive_duty = MANUAL_DRIVE_START_DUTY;
    PWM_SetDutyCycle(current_drive_duty);
}

static u8 drive_speed_increase(u8* requested_duty)
{
    *requested_duty = (u8)(current_drive_duty + MANUAL_DRIVE_STEP_DUTY);

    if(*requested_duty > MANUAL_DRIVE_MAX_DUTY)
    {
        drive_speed_reset();
        return SPEED_STEP_LIMIT;
    }

    current_drive_duty = *requested_duty;
    PWM_SetDutyCycle(current_drive_duty);

    if(current_drive_duty >= MANUAL_DRIVE_WARN_DUTY)
    {
        return SPEED_STEP_NEAR;
    }

    return SPEED_STEP_OK;
}

static void drive_speed_decrease(void)
{
    if(current_drive_duty <= (u8)(MANUAL_DRIVE_MIN_DUTY + MANUAL_DRIVE_STEP_DUTY))
    {
        current_drive_duty = MANUAL_DRIVE_MIN_DUTY;
    }
    else
    {
        current_drive_duty = (u8)(current_drive_duty - MANUAL_DRIVE_STEP_DUTY);
    }

    PWM_SetDutyCycle(current_drive_duty);
}

static void ultrasonic_init_active_sensors(void)
{
    ULTRASONIC_InitSensor(ULTRASONIC_FRONT);
    ULTRASONIC_InitSensor(ULTRASONIC_LEFT);
    ULTRASONIC_InitSensor(ULTRASONIC_RIGHT);
}

static void ultrasonic_sample_active_sensors(void)
{
    u16 front_samples[ULTRASONIC_SAMPLE_COUNT];
    u16 left_samples[ULTRASONIC_SAMPLE_COUNT];
    u16 right_samples[ULTRASONIC_SAMPLE_COUNT];
    u8  front_valid = 0U;
    u8  left_valid = 0U;
    u8  right_valid = 0U;
    u8  status;
    u16 pulse_ticks;
    u16 sample_cm;
    u8  i;

    for(i = 0U; i < ULTRASONIC_SAMPLE_COUNT; i++)
    {
        sample_cm = ULTRASONIC_ReadDetailed(ULTRASONIC_FRONT, &status, &pulse_ticks);
        (void)pulse_ticks;
        ULTRASONIC_AddValidSample(front_samples, &front_valid, sample_cm, status);
        DELAY_WITH_COMMAND_CHECKS((u8)(ULTRASONIC_INTER_PING_MS / MANUAL_DELAY_SLICE_MS));

        sample_cm = ULTRASONIC_ReadDetailed(ULTRASONIC_LEFT, &status, &pulse_ticks);
        (void)pulse_ticks;
        ULTRASONIC_AddValidSample(left_samples, &left_valid, sample_cm, status);
        DELAY_WITH_COMMAND_CHECKS((u8)(ULTRASONIC_INTER_PING_MS / MANUAL_DELAY_SLICE_MS));

        sample_cm = ULTRASONIC_ReadDetailed(ULTRASONIC_RIGHT, &status, &pulse_ticks);
        (void)pulse_ticks;
        ULTRASONIC_AddValidSample(right_samples, &right_valid, sample_cm, status);
        DELAY_WITH_COMMAND_CHECKS((u8)(ULTRASONIC_INTER_PING_MS / MANUAL_DELAY_SLICE_MS));
    }

    latest_front_cm = ULTRASONIC_MedianOrNoEcho(front_samples, front_valid);
    latest_left_cm = ULTRASONIC_MedianOrNoEcho(left_samples, left_valid);
    latest_right_cm = ULTRASONIC_MedianOrNoEcho(right_samples, right_valid);
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
            *cm = ULTRASONIC_NO_OBJ;
            return 0U;
    }

    return (u8)((*cm != ULTRASONIC_NO_OBJ) && (*cm <= ULTRASONIC_OBSTACLE_BLOCK_CM));
}

static void enforce_active_obstacle_stop(void)
{
    u16 blocked_cm = ULTRASONIC_NO_OBJ;

    if(command_block_distance(active_drive_cmd, &blocked_cm))
    {
        MOTOR_Stop();
        PROTOCOL_WRITE_OBSTACLE_WARN("STOP", active_drive_cmd, blocked_cm);
        active_drive_cmd = 'S';
    }
}

static void protocol_write_str(const char* s)
{
    while(*s != '\0')
    {
        UART_Write((u8)*s);
        s++;
    }
}

static void protocol_write_u16(u16 v)
{
    char buf[6];
    s8 i = 0;
    s8 j;

    if(v == 0U)
    {
        UART_Write('0');
        return;
    }

    while(v > 0U && i < 5)
    {
        buf[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    }

    for(j = (s8)(i - 1); j >= 0; j--)
    {
        UART_Write((u8)buf[j]);
    }
}

void MANUAL_CONTROL_Test(void)
{
    u16 hb_tick = 0U;

    MOTOR_Init();
    drive_speed_init();
    ultrasonic_init_active_sensors();
    seatbelt_init();
    ldr_init();

    UART_TX_Init();
    UART_RX_Init();

    PROTOCOL_WRITE_BOOT();
    PROTOCOL_WRITE_SPEED_TELEMETRY();

    while(1)
    {
        DRAIN_RX_COMMANDS();

        ultrasonic_sample_active_sensors();
        enforce_active_obstacle_stop();

        ldr_update_outputs();
        seatbelt_update();
        enforce_seatbelt_stop();

        DRAIN_RX_COMMANDS();
        PROTOCOL_WRITE_SENSOR_TELEMETRY();
        DRAIN_RX_COMMANDS();

        DELAY_WITH_COMMAND_CHECKS(MANUAL_POST_TELEMETRY_TICKS);

        hb_tick++;
        protocol_write_str("HB:");
        protocol_write_u16(hb_tick);
        protocol_write_str("\r\n");
    }
}
