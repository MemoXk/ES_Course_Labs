/*
 * uart_test.c
 *
 * Minimal UART TX-only diagnostic with a heartbeat LED.
 *
 * The LED is purely an "I'm alive" indicator so we can visually
 * confirm the chip is running the NEW firmware before suspecting
 * the UART path. Runs at a slow 1Hz so it's obvious.
 *
 * Sends:
 *   "BOOT\r\n"  once at startup
 *   "T:N\r\n"   every ~1 second, N = uptime counter
 *
 * Wiring:
 *   RB0 → 330Ω → LED → GND   (heartbeat: 1 blink per second)
 *   RC6 (TX, pin 25) → Logic Shifter HV1 → LV1 → Pi pin 10 (RXD)
 *   GND ↔ everything else
 *
 * Diagnostic order:
 *   1. RB0 LED blinking 1x/sec? → new firmware is running. Continue.
 *   2. Pi log shows "BOOT", "T:1", "T:2"…?  → UART path works.
 *      If yes, we know the level shifter + wiring is OK.
 *      If no, the issue is purely physical (shifter / wire / Pi pin 10).
 */

#include "uart_test.h"
#include "../MCAL/USART/USART_Interface.h"
#include "../MCAL/GPIO/GPIO_interface.h"

#define HB_PORT   GPIO_PORTB
#define HB_PIN    GPIO_PIN0

static void uart_send_str(const char *s)
{
    while(*s != '\0')
    {
        UART_Write((u8)*s);
        s++;
    }
}

static void uart_send_u16(u16 v)
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

/*
 * Short-LONG heartbeat: pip ... BLINK ... pip ... BLINK ...
 *
 * Reverse of the previous LONG-short pattern, signalling we
 * reverted the baud back to 9600.
 *
 * Distinct from earlier patterns:
 *   - solid 1Hz    = oldest uart_test
 *   - triple-blink = older uart_test (3 quick flashes)
 *   - LONG-short   = previous (4800 baud, BRGH=0)
 *   - SHORT-LONG   = THIS firmware (9600 baud revert)
 *   - 5Hz steady   = manual_control
 */
static void short_long_blink(void)
{
    /* SHORT flash */
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
    __delay_ms(100);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* small gap */
    __delay_ms(150);

    /* LONG flash */
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_HIGH);
    __delay_ms(250);
    __delay_ms(250);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* long pause so each short-LONG pair is visually distinct */
    __delay_ms(250);
    __delay_ms(250);
    __delay_ms(100);
}

void UART_Test(void)
{
    u16 n = 0;

    /* Heartbeat LED — independent of UART */
    GPIO_SetPinDirection(HB_PORT, HB_PIN, GPIO_OUTPUT);
    GPIO_SetPinValue(HB_PORT, HB_PIN, GPIO_LOW);

    /* UART TX setup (no RX needed for this test) */
    UART_TX_Init();
    uart_send_str("BOOT\r\n");

    while(1)
    {
        short_long_blink();

        /* Send heartbeat over UART once per blink burst (~0.7Hz) */
        n++;
        uart_send_str("T:");
        uart_send_u16(n);
        uart_send_str("\r\n");
    }
}
