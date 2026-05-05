#include "GPIO_interface.h"
#include "GPIO_private.h"
#include "GPIO_config.h"

#include "../../SERVICES/BIT_MATH.h"

static u8 gpio_porta_latch = GPIO_PORTA_INIT_VAL;
static u8 gpio_portb_latch = GPIO_PORTB_INIT_VAL;
static u8 gpio_portc_latch = GPIO_PORTC_INIT_VAL;
static u8 gpio_portd_latch = GPIO_PORTD_INIT_VAL;
static u8 gpio_porte_latch = GPIO_PORTE_INIT_VAL;

void GPIO_SetPinDirection(u8 Port, u8 Pin, u8 Direction)
{
    switch(Port)
    {
        case GPIO_PORTA:
            if(Direction == GPIO_OUTPUT)
                CLR_BIT(TRISA, Pin);
            else
                SET_BIT(TRISA, Pin);
        break;

        case GPIO_PORTB:
            if(Direction == GPIO_OUTPUT)
                CLR_BIT(TRISB, Pin);
            else
                SET_BIT(TRISB, Pin);
        break;

        case GPIO_PORTC:
            if(Direction == GPIO_OUTPUT)
                CLR_BIT(TRISC, Pin);
            else
                SET_BIT(TRISC, Pin);
        break;

        case GPIO_PORTD:
            if(Direction == GPIO_OUTPUT)
                CLR_BIT(TRISD, Pin);
            else
                SET_BIT(TRISD, Pin);
        break;

        case GPIO_PORTE:
            if(Direction == GPIO_OUTPUT)
                CLR_BIT(TRISE, Pin);
            else
                SET_BIT(TRISE, Pin);
        break;

        default:
        break;
    }
}


void GPIO_SetPinValue(u8 Port, u8 Pin, u8 Value)
{
    switch(Port)
    {
        case GPIO_PORTA:
            if(Value == GPIO_HIGH)
                SET_BIT(gpio_porta_latch, Pin);
            else
                CLR_BIT(gpio_porta_latch, Pin);
            PORTA = gpio_porta_latch;
        break;

        case GPIO_PORTB:
            if(Value == GPIO_HIGH)
                SET_BIT(gpio_portb_latch, Pin);
            else
                CLR_BIT(gpio_portb_latch, Pin);
            PORTB = gpio_portb_latch;
        break;

        case GPIO_PORTC:
            if(Value == GPIO_HIGH)
                SET_BIT(gpio_portc_latch, Pin);
            else
                CLR_BIT(gpio_portc_latch, Pin);
            PORTC = gpio_portc_latch;
        break;

        case GPIO_PORTD:
            if(Value == GPIO_HIGH)
                SET_BIT(gpio_portd_latch, Pin);
            else
                CLR_BIT(gpio_portd_latch, Pin);
            PORTD = gpio_portd_latch;
        break;

        case GPIO_PORTE:
            if(Value == GPIO_HIGH)
                SET_BIT(gpio_porte_latch, Pin);
            else
                CLR_BIT(gpio_porte_latch, Pin);
            PORTE = gpio_porte_latch;
        break;

        default:
        break;
    }
}


u8 GPIO_GetPinValue(u8 Port, u8 Pin)
{
    u8 Local_Value = 0;

    switch(Port)
    {
        case GPIO_PORTA:
            Local_Value = GET_BIT(PORTA, Pin);
        break;

        case GPIO_PORTB:
            Local_Value = GET_BIT(PORTB, Pin);
        break;

        case GPIO_PORTC:
            Local_Value = GET_BIT(PORTC, Pin);
        break;

        case GPIO_PORTD:
            Local_Value = GET_BIT(PORTD, Pin);
        break;

        case GPIO_PORTE:
            Local_Value = GET_BIT(PORTE, Pin);
        break;

        default:
        break;
    }

    return Local_Value;
}

void GPIO_Init(void)
{
    gpio_porta_latch = GPIO_PORTA_INIT_VAL;
    gpio_portb_latch = GPIO_PORTB_INIT_VAL;
    gpio_portc_latch = GPIO_PORTC_INIT_VAL;
    gpio_portd_latch = GPIO_PORTD_INIT_VAL;
    gpio_porte_latch = GPIO_PORTE_INIT_VAL;

    TRISA = GPIO_PORTA_DIR;
    TRISB = GPIO_PORTB_DIR;
    TRISC = GPIO_PORTC_DIR;
    TRISD = GPIO_PORTD_DIR;
    TRISE = GPIO_PORTE_DIR;

    PORTA = gpio_porta_latch;
    PORTB = gpio_portb_latch;
    PORTC = gpio_portc_latch;
    PORTD = gpio_portd_latch;
    PORTE = gpio_porte_latch;
}
