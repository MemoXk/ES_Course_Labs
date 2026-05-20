#include "ULTRASONIC_interface.h"

#define ULTRASONIC_SELECT_PINS(sensor_id, trig_port, trig_pin, echo_port, echo_pin, valid) \
    do                                                                                    \
    {                                                                                     \
        valid = 1U;                                                                        \
        switch(sensor_id)                                                                  \
        {                                                                                 \
            case ULTRASONIC_FRONT:                                                        \
                trig_port = ULTRASONIC_FRONT_TRIG_PORT;                                   \
                trig_pin  = ULTRASONIC_FRONT_TRIG_PIN;                                    \
                echo_port = ULTRASONIC_FRONT_ECHO_PORT;                                   \
                echo_pin  = ULTRASONIC_FRONT_ECHO_PIN;                                    \
                break;                                                                    \
            case ULTRASONIC_BACK:                                                         \
                trig_port = ULTRASONIC_BACK_TRIG_PORT;                                    \
                trig_pin  = ULTRASONIC_BACK_TRIG_PIN;                                     \
                echo_port = ULTRASONIC_BACK_ECHO_PORT;                                    \
                echo_pin  = ULTRASONIC_BACK_ECHO_PIN;                                     \
                break;                                                                    \
            case ULTRASONIC_LEFT:                                                         \
                trig_port = ULTRASONIC_LEFT_TRIG_PORT;                                    \
                trig_pin  = ULTRASONIC_LEFT_TRIG_PIN;                                     \
                echo_port = ULTRASONIC_LEFT_ECHO_PORT;                                    \
                echo_pin  = ULTRASONIC_LEFT_ECHO_PIN;                                     \
                break;                                                                    \
            case ULTRASONIC_RIGHT:                                                        \
                trig_port = ULTRASONIC_RIGHT_TRIG_PORT;                                   \
                trig_pin  = ULTRASONIC_RIGHT_TRIG_PIN;                                    \
                echo_port = ULTRASONIC_RIGHT_ECHO_PORT;                                   \
                echo_pin  = ULTRASONIC_RIGHT_ECHO_PIN;                                    \
                break;                                                                    \
            case ULTRASONIC_EXTRA:                                                        \
                trig_port = ULTRASONIC_EXTRA_TRIG_PORT;                                   \
                trig_pin  = ULTRASONIC_EXTRA_TRIG_PIN;                                    \
                echo_port = ULTRASONIC_EXTRA_ECHO_PORT;                                   \
                echo_pin  = ULTRASONIC_EXTRA_ECHO_PIN;                                    \
                break;                                                                    \
            default:                                                                      \
                valid = 0U;                                                               \
                break;                                                                    \
        }                                                                                 \
    } while(0)

void ULTRASONIC_Init(void)
{
    TIMER1_Init();

    ULTRASONIC_InitSensor(ULTRASONIC_FRONT);
    ULTRASONIC_InitSensor(ULTRASONIC_BACK);
    ULTRASONIC_InitSensor(ULTRASONIC_LEFT);
    ULTRASONIC_InitSensor(ULTRASONIC_RIGHT);
    ULTRASONIC_InitSensor(ULTRASONIC_EXTRA);
}

void ULTRASONIC_InitSensor(u8 sensor_id)
{
    u8 trig_port;
    u8 trig_pin;
    u8 echo_port;
    u8 echo_pin;

    TIMER1_Init();

    u8 valid;

    ULTRASONIC_SELECT_PINS(sensor_id, trig_port, trig_pin, echo_port, echo_pin, valid);
    if(!valid)
    {
        return;
    }

    GPIO_SetPinDirection(trig_port, trig_pin, GPIO_OUTPUT);
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_LOW);
    GPIO_SetPinDirection(echo_port, echo_pin, GPIO_INPUT);
}

u16 ULTRASONIC_ReadDetailed(u8 sensor_id, u8* status, u16* pulse_ticks)
{
    u8  trig_port;
    u8  trig_pin;
    u8  echo_port;
    u8  echo_pin;
    u8  local_status;
    u16 local_ticks;
    u16 width_ticks = 0U;

    if(status == NULL_PTR)
    {
        status = &local_status;
    }
    if(pulse_ticks == NULL_PTR)
    {
        pulse_ticks = &local_ticks;
    }

    *status = 'O';
    *pulse_ticks = 0U;

    u8 valid;

    ULTRASONIC_SELECT_PINS(sensor_id, trig_port, trig_pin, echo_port, echo_pin, valid);
    if(!valid)
    {
        *status = 'I';
        return ULTRASONIC_NO_OBJ;
    }

    TIMER1_Stop();
    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_HIGH)
    {
        if(TIMER1_GetValue() >= ULTRASONIC_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'H';
            return ULTRASONIC_NO_OBJ;
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
        if(TIMER1_GetValue() >= ULTRASONIC_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'N';
            return ULTRASONIC_NO_OBJ;
        }
    }

    TIMER1_Stop();
    TIMER1_Reset();
    TIMER1_Start();
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_HIGH)
    {
        width_ticks = TIMER1_GetValue();
        if(width_ticks >= ULTRASONIC_TIMEOUT_TICKS)
        {
            TIMER1_Stop();
            *status = 'T';
            *pulse_ticks = width_ticks;
            return ULTRASONIC_NO_OBJ;
        }
    }
    TIMER1_Stop();

    *pulse_ticks = width_ticks;

    if(width_ticks < ULTRASONIC_MIN_WIDTH_TICKS)
    {
        *status = 'S';
        return ULTRASONIC_NO_OBJ;
    }

    return (u16)(((u32)width_ticks + 72UL) / (u32)ULTRASONIC_TICKS_PER_CM);
}

u16 ULTRASONIC_GetDistance(u8 sensor_id)
{
    return ULTRASONIC_ReadDetailed(sensor_id, NULL_PTR, NULL_PTR);
}

void ULTRASONIC_AddValidSample(u16 samples[], u8* count, u16 sample_cm, u8 status)
{
    if(status == 'O' && sample_cm != ULTRASONIC_NO_OBJ && sample_cm <= ULTRASONIC_MAX_VALID_CM)
    {
        samples[*count] = sample_cm;
        (*count)++;
    }
}

u16 ULTRASONIC_MedianOrNoEcho(u16 samples[], u8 count)
{
    u16 temp;
    u8  i;
    u8  j;

    if(count < ULTRASONIC_MIN_VALID)
    {
        return ULTRASONIC_NO_OBJ;
    }

    for(i = 0U; i < count; i++)
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
