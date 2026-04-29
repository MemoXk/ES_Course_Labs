#include "USART_Interface.h"


/* =================================
   Global Pointer To Callback
================================= */

void (*UART_Callback)(u8) = 0;

/* =================================
   RX Initialization
================================= */

void UART_RX_Init(void)
{

    SET_BIT(TXSTA , BRGH_BIT);              /* High Speed Mode */

    SPBRG = UART_SPBRG_VALUE;          /* Baud rate from config */

    CLR_BIT(TXSTA , SYNC_BIT);      // Asynchronous Mode

    SET_BIT(RCSTA , SPEN_BIT);      // Enable Serial Port

    SET_BIT(RCSTA , CREN_BIT);      // Continuous Receive

    SET_BIT(PIE1 , RCIE_BIT);       // Enable UART RX Interrupt

    SET_BIT(INTCON , PEIE_BIT);     // Peripheral Interrupt Enable
    SET_BIT(INTCON , GIE_BIT);      // Global Interrupt Enable
}

/* =================================
   TX Initialization
================================= */

void UART_TX_Init(void)
{

    SET_BIT(TXSTA , BRGH_BIT);              /* High Speed */

    SPBRG = UART_SPBRG_VALUE;          /* Baud rate from config */

    CLR_BIT(TXSTA , SYNC_BIT);      // Asynchronous Mode

    SET_BIT(RCSTA , SPEN_BIT);      // Enable Serial Port

    SET_BIT(TXSTA , TXEN_BIT);      // Enable Transmission
}

/* =================================
   Send Byte
================================= */

void UART_Write(u8 Data)
{

    while(!GET_BIT(TXSTA , TRMT_BIT));   // Wait until TX empty

    TXREG = Data;
}

/* =================================
   Receive Byte (Polling)
================================= */

u8 UART_Read(void)
{

    while(!GET_BIT(PIR1 , RCIF_BIT));    // Wait for data

    return RCREG;
}

/* =================================
   TX Buffer Status
================================= */

u8 UART_TX_Empty(void)
{

    return GET_BIT(TXSTA , TRMT_BIT);
}

/* =================================
   Callback Setter
================================= */

void UART_SetCallback(void (*Callback)(u8))
{

    if(Callback != 0)
    {
        UART_Callback = Callback;
    }

}

void UART_ISR(void)
{

    u8 UART_data = RCREG;   //
    if(UART_Callback != 0)
    {
        UART_Callback(UART_data);   //
    }

}


/* =================================
   ISR Handler
================================= */

/*
void interrupt()
{

   if(GET_BIT(PIR1 , RCIF))
    {

        if(UART_Callback != 0)
        {
            UART_Callback();   // Call user function
        }

    }

}
  */