#include "ULTRASONIC_interface.h"

/* =========================================================
   pin_lookup
   Maps sensor_id to its TRIG/ECHO port and pin numbers.
   Returns 0 on success, 1 if sensor_id is invalid.
========================================================= */
static u8 pin_lookup(u8 sensor_id,
                     u8 *trig_port, u8 *trig_pin,
                     u8 *echo_port, u8 *echo_pin)
{
    switch(sensor_id)
    {
        case ULTRASONIC_FRONT:
            *trig_port = ULTRASONIC_FRONT_TRIG_PORT;
            *trig_pin  = ULTRASONIC_FRONT_TRIG_PIN;
            *echo_port = ULTRASONIC_FRONT_ECHO_PORT;
            *echo_pin  = ULTRASONIC_FRONT_ECHO_PIN;
            break;
        case ULTRASONIC_BACK:
            *trig_port = ULTRASONIC_BACK_TRIG_PORT;
            *trig_pin  = ULTRASONIC_BACK_TRIG_PIN;
            *echo_port = ULTRASONIC_BACK_ECHO_PORT;
            *echo_pin  = ULTRASONIC_BACK_ECHO_PIN;
            break;
        case ULTRASONIC_LEFT:
            *trig_port = ULTRASONIC_LEFT_TRIG_PORT;
            *trig_pin  = ULTRASONIC_LEFT_TRIG_PIN;
            *echo_port = ULTRASONIC_LEFT_ECHO_PORT;
            *echo_pin  = ULTRASONIC_LEFT_ECHO_PIN;
            break;
        case ULTRASONIC_RIGHT:
            *trig_port = ULTRASONIC_RIGHT_TRIG_PORT;
            *trig_pin  = ULTRASONIC_RIGHT_TRIG_PIN;
            *echo_port = ULTRASONIC_RIGHT_ECHO_PORT;
            *echo_pin  = ULTRASONIC_RIGHT_ECHO_PIN;
            break;
        default:
            return 1u;
    }
    return 0u;
}

/* =========================================================
   ULTRASONIC_Init
========================================================= */
void ULTRASONIC_Init(void)
{
    /* TRIG pins → output, start LOW */
    GPIO_SetPinDirection(ULTRASONIC_FRONT_TRIG_PORT, ULTRASONIC_FRONT_TRIG_PIN, GPIO_OUTPUT);
    GPIO_SetPinDirection(ULTRASONIC_BACK_TRIG_PORT,  ULTRASONIC_BACK_TRIG_PIN,  GPIO_OUTPUT);
    GPIO_SetPinDirection(ULTRASONIC_LEFT_TRIG_PORT,  ULTRASONIC_LEFT_TRIG_PIN,  GPIO_OUTPUT);
    GPIO_SetPinDirection(ULTRASONIC_RIGHT_TRIG_PORT, ULTRASONIC_RIGHT_TRIG_PIN, GPIO_OUTPUT);

    GPIO_SetPinValue(ULTRASONIC_FRONT_TRIG_PORT, ULTRASONIC_FRONT_TRIG_PIN, GPIO_LOW);
    GPIO_SetPinValue(ULTRASONIC_BACK_TRIG_PORT,  ULTRASONIC_BACK_TRIG_PIN,  GPIO_LOW);
    GPIO_SetPinValue(ULTRASONIC_LEFT_TRIG_PORT,  ULTRASONIC_LEFT_TRIG_PIN,  GPIO_LOW);
    GPIO_SetPinValue(ULTRASONIC_RIGHT_TRIG_PORT, ULTRASONIC_RIGHT_TRIG_PIN, GPIO_LOW);

    /* ECHO pins → input */
    GPIO_SetPinDirection(ULTRASONIC_FRONT_ECHO_PORT, ULTRASONIC_FRONT_ECHO_PIN, GPIO_INPUT);
    GPIO_SetPinDirection(ULTRASONIC_BACK_ECHO_PORT,  ULTRASONIC_BACK_ECHO_PIN,  GPIO_INPUT);
    GPIO_SetPinDirection(ULTRASONIC_LEFT_ECHO_PORT,  ULTRASONIC_LEFT_ECHO_PIN,  GPIO_INPUT);
    GPIO_SetPinDirection(ULTRASONIC_RIGHT_ECHO_PORT, ULTRASONIC_RIGHT_ECHO_PIN, GPIO_INPUT);
}

/* =========================================================
   ULTRASONIC_GetDistance
   Returns distance in cm, or ULTRASONIC_NO_OBJ if no echo.
   Timer1 is used for pulse-width measurement:
     @ 20 MHz, prescaler 1:2 → 1 tick = 0.4 µs → 145 ticks/cm
========================================================= */
u16 ULTRASONIC_GetDistance(u8 sensor_id)
{
    u8  trig_port, trig_pin, echo_port, echo_pin;
    u16 wait;
    u16 ticks;

    if(pin_lookup(sensor_id, &trig_port, &trig_pin, &echo_port, &echo_pin) != 0u)
    {
        return ULTRASONIC_NO_OBJ;
    }

    /* Send 10 µs trigger pulse */
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_LOW);
    Delay_us(2);
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_HIGH);
    Delay_us(10);
    GPIO_SetPinValue(trig_port, trig_pin, GPIO_LOW);

    /* Wait for echo to go HIGH (with timeout) */
    wait = 0u;
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_LOW)
    {
        wait++;
        if(wait >= ULTRASONIC_ECHO_WAIT_MAX)
        {
            return ULTRASONIC_NO_OBJ;
        }
    }

    /* Start Timer1: prescaler 1:2, internal clock */
    TMR1H = 0u;
    TMR1L = 0u;
    T1CON = T1CON_START;

    /* Measure echo HIGH duration, stop on overflow guard (~400 cm) */
    while(GPIO_GetPinValue(echo_port, echo_pin) == GPIO_HIGH)
    {
        if(TMR1H >= ULTRASONIC_OVERFLOW_H)
        {
            T1CON = T1CON_STOP;
            return ULTRASONIC_NO_OBJ;
        }
    }

    T1CON = T1CON_STOP;

    ticks = ((u16)TMR1H << 8) | (u16)TMR1L;

    return (u16)(ticks / ULTRASONIC_TICKS_PER_CM);
}
