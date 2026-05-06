#ifndef UART_PRIVATE_H
#define UART_PRIVATE_H

/* ================= Register Addresses (PIC16F877A) ================= */

#ifndef TXSTA
#define TXSTA   (*(volatile unsigned char*)0x98)
#endif
#ifndef RCSTA
#define RCSTA   (*(volatile unsigned char*)0x18)
#endif
#ifndef SPBRG
#define SPBRG   (*(volatile unsigned char*)0x99)
#endif
#ifndef TXREG
#define TXREG   (*(volatile unsigned char*)0x19)
#endif
#ifndef RCREG
#define RCREG   (*(volatile unsigned char*)0x1A)
#endif

#ifndef PIR1
#define PIR1    (*(volatile unsigned char*)0x0C)
#endif

#ifndef PIE1
#define PIE1    (*(volatile unsigned char*)0x8C)
#endif

#ifndef INTCON
#define INTCON  (*(volatile unsigned char*)0x0B)
#endif
#ifndef TRISC
#define TRISC   (*(volatile unsigned char*)0x87)
#endif

/* ================= TXSTA Bit positions ================= */

#define TXEN_BIT   5
#define BRGH_BIT   2
#define SYNC_BIT   4
#define TRMT_BIT   1

/* ================= RCSTA Bit positions ================= */

#define SPEN_BIT   7
#define CREN_BIT   4
#define FERR_BIT   2   /* Framing Error  */
#define OERR_BIT   1   /* Overrun Error  */

/* ================= PIR1 Bit positions ================= */

#define RCIF_BIT   5

/* ================= PIE1 Bit positions ================= */

#define RCIE_BIT   5

/* ================= INTCON Bit positions ================= */

#define PEIE_BIT   6
#define GIE_BIT    7

/* ================= UART pin directions ================= */

#define UART_TX_TRIS_BIT  6
#define UART_RX_TRIS_BIT  7

#endif
