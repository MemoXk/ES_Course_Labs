#include "USART_Interface.h"


/* =================================
   Global Pointer To Callback
   (kept for API compatibility;
    UART_ISR no longer calls it —
    see note below)
================================= */

void (*UART_Callback)(u8) = 0;

/* =================================
   ISR-safe RX ring buffer.
   UART_ISR enqueues command bytes;
   the foreground drains them later.
   This avoids losing quick commands
   while the PIC is measuring sensors
   or writing telemetry.
================================= */

#define UART_RX_BUFFER_SIZE 8U

static volatile u8 UART_rx_buffer[UART_RX_BUFFER_SIZE];
static volatile u8 UART_rx_head = 0;
static volatile u8 UART_rx_tail = 0;
static volatile u8 UART_rx_count = 0;
static volatile u16 UART_rx_isr_count = 0;
static volatile u16 UART_rx_byte_count = 0;
static volatile u16 UART_rx_overrun_count = 0;
static volatile u16 UART_rx_framing_count = 0;
static volatile u8 UART_rx_last_byte = 0;

static u16 UART_ReadCounterAtomic(volatile u16* counter)
{
    u16 value;
    u8 gie_was_enabled;

    gie_was_enabled = GET_BIT(INTCON, GIE_BIT);
    CLR_BIT(INTCON, GIE_BIT);
    value = *counter;
    if(gie_was_enabled)
    {
        SET_BIT(INTCON, GIE_BIT);
    }
    return value;
}

/* =================================
   RX Initialization
================================= */

void UART_RX_Init(void)
{
    u8 i;

    SET_BIT(TRISC, UART_RX_TRIS_BIT);
    for(i = 0; i < UART_RX_BUFFER_SIZE; i++)
    {
        UART_rx_buffer[i] = 0;
    }
    UART_rx_head = 0;
    UART_rx_tail = 0;
    UART_rx_count = 0;

#if (UART_HIGH_SPEED == 1)
    SET_BIT(TXSTA , BRGH_BIT);          /* High Speed Mode (BRGH=1) */
#else
    CLR_BIT(TXSTA , BRGH_BIT);          /* Low Speed Mode  (BRGH=0) */
#endif

    SPBRG = UART_SPBRG_VALUE;          /* Baud rate from config */

    CLR_BIT(TXSTA , SYNC_BIT);      // Asynchronous Mode

    SET_BIT(RCSTA , SPEN_BIT);      // Enable Serial Port

    CLR_BIT(RCSTA , CREN_BIT);      // Reset receiver before enabling
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

    UART_rx_isr_count++;

    /* Recover from Overrun Error: toggle CREN to reset the receiver.
     * If OERR sets the hardware refuses further bytes until CREN is
     * cleared and re-enabled.                                         */
    if(GET_BIT(RCSTA, OERR_BIT))
    {
        UART_rx_overrun_count++;
        CLR_BIT(RCSTA, CREN_BIT);
        SET_BIT(RCSTA, CREN_BIT);
        return;
    }

    /* Reading RCREG clears RCIF and any framing error flag.
     * We write directly to the volatile RX ring — NO function-pointer
     * call.  On PIC16 a computed call (via pointer) inside an ISR
     * requires correct PCLATH setup at runtime; XC8's code generation
     * for that case is fragile and silently misfired here.
     * Direct enqueue into volatile storage is always safe.
     *
     * Line-ending bytes ('\r', '\n') are consumed but discarded.
     * The Pi appends '\n' to every command ("F\n"), so filtering keeps
     * line endings out of the command queue. */
    ferr = GET_BIT(RCSTA, FERR_BIT);
    rx_byte = RCREG;
    UART_rx_last_byte = rx_byte;
    if(ferr)
    {
        UART_rx_framing_count++;
        return;
    }
    if(rx_byte == '\r' || rx_byte == '\n') { return; }

    if(UART_rx_count < UART_RX_BUFFER_SIZE)
    {
        UART_rx_buffer[UART_rx_head] = rx_byte;
        UART_rx_head++;
        if(UART_rx_head >= UART_RX_BUFFER_SIZE)
        {
            UART_rx_head = 0;
        }
        UART_rx_count++;
    }
    else
    {
        UART_rx_overrun_count++;
    }
    UART_rx_byte_count++;
}

/* =================================
   ISR-driven RX getters
   Call these from the main loop.
================================= */

u8 UART_RX_IsReady(void)
{
    return (u8)(UART_rx_count > 0U);
}

u8 UART_RX_GetByte(void)
{
    u8 data = 0;
    u8 gie_was_enabled;

    gie_was_enabled = GET_BIT(INTCON, GIE_BIT);
    CLR_BIT(INTCON, GIE_BIT);

    if(UART_rx_count > 0U)
    {
        data = UART_rx_buffer[UART_rx_tail];
        UART_rx_tail++;
        if(UART_rx_tail >= UART_RX_BUFFER_SIZE)
        {
            UART_rx_tail = 0;
        }
        UART_rx_count--;
    }

    if(gie_was_enabled)
    {
        SET_BIT(INTCON, GIE_BIT);
    }

    return data;
}

u16 UART_RX_GetIsrCount(void)
{
    return UART_ReadCounterAtomic(&UART_rx_isr_count);
}

u16 UART_RX_GetByteCount(void)
{
    return UART_ReadCounterAtomic(&UART_rx_byte_count);
}

u16 UART_RX_GetOverrunCount(void)
{
    return UART_ReadCounterAtomic(&UART_rx_overrun_count);
}

u16 UART_RX_GetFramingCount(void)
{
    return UART_ReadCounterAtomic(&UART_rx_framing_count);
}

u8 UART_RX_GetLastByte(void)
{
    return UART_rx_last_byte;
}

u8 UART_RX_GetReadyFlag(void)
{
    return UART_rx_count;
}

u8 UART_Debug_ReadRCSTA(void)
{
    return RCSTA;
}

u8 UART_Debug_ReadPIR1(void)
{
    return PIR1;
}

u8 UART_Debug_ReadPIE1(void)
{
    return PIE1;
}

u8 UART_Debug_ReadINTCON(void)
{
    return INTCON;
}

u8 UART_Debug_ReadTRISC(void)
{
    return TRISC;
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
