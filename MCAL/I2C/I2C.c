#include "I2C_Interface.h"

/* =========================================================
   I2C_MasterInit
========================================================= */

void I2C_MasterInit(void)
{
    /* Configure SCL (RC3) and SDA (RC4) as inputs (MSSP drives open-drain) */
    GPIO_SetPinDirection(GPIO_PORTC, GPIO_PIN3, GPIO_INPUT);
    GPIO_SetPinDirection(GPIO_PORTC, GPIO_PIN4, GPIO_INPUT);

    /* Set baud rate */
    SSPADD = I2C_SSPADD_VALUE;

    /* Configure SSPCON: I2C master mode, SSP enabled */
    SSPCON  = (u8)(I2C_MASTER_MODE | (1U << SSPEN_BIT));

    /* Clear SSPCON2 */
    SSPCON2 = 0x00;

    /* Clear SSPSTAT */
    SSPSTAT = 0x00;

    /* Clear interrupt flag */
    CLR_BIT(PIR1, SSPIF_BIT);
}

/* =========================================================
   I2C_Start
========================================================= */

void I2C_Start(void)
{
    CLR_BIT(PIR1, SSPIF_BIT);
    SET_BIT(SSPCON2, SEN_BIT);              /* Initiate START condition */
    while(!GET_BIT(PIR1, SSPIF_BIT)) { ; } /* Wait for completion      */
    CLR_BIT(PIR1, SSPIF_BIT);
}

/* =========================================================
   I2C_Stop
========================================================= */

void I2C_Stop(void)
{
    CLR_BIT(PIR1, SSPIF_BIT);
    SET_BIT(SSPCON2, PEN_BIT);              /* Initiate STOP condition */
    while(!GET_BIT(PIR1, SSPIF_BIT)) { ; }
    CLR_BIT(PIR1, SSPIF_BIT);
}

/* =========================================================
   I2C_RepeatedStart
========================================================= */

void I2C_RepeatedStart(void)
{
    CLR_BIT(PIR1, SSPIF_BIT);
    SET_BIT(SSPCON2, RSEN_BIT);             /* Initiate repeated START */
    while(!GET_BIT(PIR1, SSPIF_BIT)) { ; }
    CLR_BIT(PIR1, SSPIF_BIT);
}

/* =========================================================
   I2C_WriteByte
   Returns 0 = ACK received, 1 = NACK received
========================================================= */

u8 I2C_WriteByte(u8 Data)
{
    CLR_BIT(PIR1, SSPIF_BIT);
    SSPBUF = Data;                          /* Load byte — starts transmission */
    while(!GET_BIT(PIR1, SSPIF_BIT)) { ; }
    CLR_BIT(PIR1, SSPIF_BIT);

    return GET_BIT(SSPCON2, ACKSTAT_BIT);  /* 0=ACK, 1=NACK */
}

/* =========================================================
   I2C_ReadByte
   ack: I2C_SEND_ACK=1, I2C_SEND_NACK=0
========================================================= */

u8 I2C_ReadByte(u8 ack)
{
    u8 rxByte;

    CLR_BIT(PIR1, SSPIF_BIT);
    SET_BIT(SSPCON2, RCEN_BIT);             /* Enable receive mode      */
    while(!GET_BIT(PIR1, SSPIF_BIT)) { ; } /* Wait for byte received   */
    CLR_BIT(PIR1, SSPIF_BIT);

    rxByte = SSPBUF;

    /* Send ACK or NACK */
    if(ack == I2C_SEND_ACK)
    {
        CLR_BIT(SSPCON2, ACKDT_BIT);        /* ACKDT=0 → send ACK  */
    }
    else
    {
        SET_BIT(SSPCON2, ACKDT_BIT);        /* ACKDT=1 → send NACK */
    }

    CLR_BIT(PIR1, SSPIF_BIT);
    SET_BIT(SSPCON2, ACKEN_BIT);            /* Initiate ACK sequence    */
    while(!GET_BIT(PIR1, SSPIF_BIT)) { ; }
    CLR_BIT(PIR1, SSPIF_BIT);

    return rxByte;
}

/* =========================================================
   I2C_Disable
========================================================= */

void I2C_Disable(void)
{
    CLR_BIT(SSPCON, SSPEN_BIT);
}