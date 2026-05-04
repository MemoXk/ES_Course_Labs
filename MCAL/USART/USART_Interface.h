#ifndef USART_INTERFACE_H
#define USART_INTERFACE_H

#include "USART_Private.h"
#include "USART_Config.h"
#include "../../SERVICES/STD_TYPES.h"
#include "../../SERVICES/BIT_MATH.h"

/* Initialization */
void UART_RX_Init(void);
void UART_TX_Init(void);

/* Data Operations */
void UART_Write(u8 Data);
u8 UART_Read(void);

/* Status */
u8 UART_TX_Empty(void);

/* Polled RX (no interrupt required) */
void UART_RX_Enable_Polled(void);
u8   UART_RX_HasData(void);

/* ISR-driven RX (use with UART_RX_Init)
 * ISR writes the byte; main loop reads these getters.        */
u8   UART_RX_IsReady(void);
u8   UART_RX_GetByte(void);

void UART_SetCallback(void (*Callback)(u8));
void UART_ISR(void);

#endif
