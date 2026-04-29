#ifndef ADC_PRIVATE_H
#define ADC_PRIVATE_H

#include "../../SERVICES/STD_TYPES.h"

/*
 * PIC16F877A ADC registers
 *
 * Available channels: AN0–AN7
 *   AN0–AN5 → RA0–RA5
 *   AN6–AN7 → RE1–RE2  (only on 877A 40-pin package)
 */

#ifndef ADCON0
#define ADCON0      (*(volatile u8*)0x1F)   /* ADC control register 0 (bank 0) */
#endif
#ifndef ADCON1
#define ADCON1      (*(volatile u8*)0x9F)   /* ADC control register 1 (bank 1) */
#endif
#ifndef ADRESH
#define ADRESH      (*(volatile u8*)0x1E)   /* ADC result high byte  (bank 0)  */
#endif
#ifndef ADRESL
#define ADRESL      (*(volatile u8*)0x9E)   /* ADC result low byte   (bank 1)  */
#endif

/* PIR1 — shared */
#ifndef PIR1
#define PIR1        (*(volatile u8*)0x0C)
#endif

/* ================= ADCON0 bit positions ================= */
#define ADCS1_BIT   7   /* ADC clock select bit 1 */
#define ADCS0_BIT   6   /* ADC clock select bit 0 */
#define CHS2_BIT    5   /* Channel select bit 2    */
#define CHS1_BIT    4   /* Channel select bit 1    */
#define CHS0_BIT    3   /* Channel select bit 0    */
#define GO_BIT      2   /* GO/DONE: start/status   */
#define ADON_BIT    0   /* ADC module on/off       */

/* ================= ADCON1 bit positions ================= */
#define ADFM_BIT    7   /* Result format: 1=right-justify */

/* ================= PIR1 bit positions ================= */
#define ADIF_BIT    6   /* ADC interrupt flag */

/* ================= ADCS clock selections ================= */
/* ADCS1:ADCS0 in bits 7:6 of ADCON0                        */
#define ADC_CLK_FOSC_2    0x00  /* Fosc/2   — often too fast; check Tad min */
#define ADC_CLK_FOSC_8    0x01  /* Fosc/8                         */
#define ADC_CLK_FOSC_32   0x02  /* Fosc/32  — common choice @ 4–8 MHz */
#define ADC_CLK_FRC       0x03  /* Internal RC oscillator         */

/* ================= Channel masks ================= */
#define ADC_CHANNEL_MASK  0x38  /* bits 5:3 of ADCON0 */

/* ================= ADCON1 port configuration ================= */
/*
 * PCFG3:PCFG0 (bits 3:0) configure which pins are analog.
 * 0x00 = all PortA + RE0-RE2 analog, Vref = Vdd/Vss
 * 0x0E = only AN0 analog, rest digital (good for single-channel use)
 * Set ADFM=1 for right-justified 10-bit result.
 */
#define ADC_ADCON1_ALL_ANALOG   0x80  /* ADFM=1, all pins analog */
#define ADC_ADCON1_AN0_ONLY     0x8E  /* ADFM=1, only AN0 analog */

#endif