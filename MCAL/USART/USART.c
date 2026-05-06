#include "USART_Interface.h"


/* =================================
   Global Pointer To Callback
   (kept for API compatibility;
    UART_ISR no longer calls it —
    see note below)
================================= */

void (*UART_Callback)(u8) = 0;

/* =================================
   ISR-safe RX flag pair
   Written by UART_ISR (inside ISR),
   read by main loop via API below.
   Avoids function-pointer computed
   calls on PIC16 which are
   unreliable inside ISR context.
================================= */

static volatile u8 UART_rx_data  = 0;
static volatile u8 UART_rx_ready = 0;

static void UART_RX_ClearHardware(void)
{
    volatile u8 discard;
    u8 guard = 0;

    UART_rx_data = 0;
    UART_rx_ready = 0;

    if(GET_BIT(RCSTA, OERR_BIT))
    {
        CLR_BIT(RCSTA, CREN_BIT);
        SET_BIT(RCSTA, CREN_BIT);
    }

    while(GET_BIT(PIR1, RCIF_BIT) && guard < 2U)
    {
        discard = RCREG;
        (void)discard;
        guard++;
    }
}

/* =================================
   RX Initialization
================================= */

void UART_RX_Init(void)
{
    SET_BIT(TRISC, UART_RX_TRIS_BIT);

#if (UART_HIGH_SPEED == 1)
    SET_BIT(TXSTA , BRGH_BIT);          /* High Speed Mode (BRGH=1) */
#else
    CLR_BIT(TXSTA , BRGH_BIT);          /* Low Speed Mode  (BRGH=0) */
#endif

    SPBRG = UART_SPBRG_VALUE;          /* Baud rate from config */

    CLR_BIT(TXSTA , SYNC_BIT);      // Asynchronous Mode

    SET_BIT(RCSTA , SPEN_BIT);      // Enable Serial Port

    CLR_BIT(RCSTA , CREN_BIT);      // Reset receiver before enabling
    UART_RX_ClearHardware();
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
    CLR_BIT(TRISC, UART_TX_TRIS_BIT);

#if (UART_HIGH_SPEED == 1)
    SET_BIT(TXSTA , BRGH_BIT);          /* High Speed Mode (BRGH=1) */
#else
    CLR_BIT(TXSTA , BRGH_BIT);          /* Low Speed Mode  (BRGH=0) */
#endif

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
   Polled RX — no interrupt needed
================================= */

void UART_RX_Enable_Polled(void)
{
    /* SPEN is already set by UART_TX_Init().
     * Just enable the receiver — no RCIE/PEIE/GIE needed. */
    SET_BIT(TRISC, UART_RX_TRIS_BIT);
    CLR_BIT(RCSTA , CREN_BIT);
    UART_RX_ClearHardware();
    SET_BIT(RCSTA , CREN_BIT);
}

u8 UART_RX_HasData(void)
{
    /* Auto-recover from overrun: if OERR is set the receiver locks up
     * and will never set RCIF again until CREN is toggled.            */
    if(GET_BIT(RCSTA , OERR_BIT))
    {
        CLR_BIT(RCSTA , CREN_BIT);
        SET_BIT(RCSTA , CREN_BIT);
        return 0;
    }
    return GET_BIT(PIR1 , RCIF_BIT);
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
    u8 rx_byte;
    u8 ferr;

    /* Recover from Overrun Error: toggle CREN to reset the receiver.
     * If OERR sets the hardware refuses further bytes until CREN is
     * cleared and re-enabled.                                         */
    if(GET_BIT(RCSTA, OERR_BIT))
    {
        CLR_BIT(RCSTA, CREN_BIT);
        SET_BIT(RCSTA, CREN_BIT);
    }

    /* Reading RCREG clears RCIF and any framing error flag.
     * We write directly to the volatile flag pair — NO function-pointer
     * call.  On PIC16 a computed call (via pointer) inside an ISR
     * requires correct PCLATH setup at runtime; XC8's code generation
     * for that case is fragile and silently misfired here.
     * Direct assignment to a volatile variable is always safe.
     *
     * Line-ending bytes ('\r', '\n') are consumed but discarded.
     * The Pi appends '\n' to every command ("F\n").  Without this
     * filter the '\n' ISR fires ~1 ms after the command byte and
     * overwrites UART_rx_data before the main loop has a chance to
     * read it, so the loop always sees '\n' → default case → no ACK. */
    ferr = GET_BIT(RCSTA, FERR_BIT);
    rx_byte = RCREG;
    if(ferr) { return; }
    if(rx_byte == '\r' || rx_byte == '\n') { return; }

    UART_rx_data = rx_byte;
    UART_rx_ready = 1;
}

/* =================================
   ISR-driven RX getters
   Call these from the main loop.
================================= */

u8 UART_RX_IsReady(void)
{
    return UART_rx_ready;
}

u8 UART_RX_GetByte(void)
{
    u8 data;
    u8 gie_was_enabled;

    gie_was_enabled = GET_BIT(INTCON, GIE_BIT);
    CLR_BIT(INTCON, GIE_BIT);

    data = UART_rx_data;
    UART_rx_ready = 0;

    if(gie_was_enabled)
    {
        SET_BIT(INTCON, GIE_BIT);
    }

    return data;
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
