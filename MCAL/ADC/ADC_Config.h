#ifndef ADC_CONFIG_H
#define ADC_CONFIG_H

/*
 * ADC Configuration for PIC16F877A @ 20 MHz
 *
 * Clock: Fosc/32 (PIC16F877A has only Fosc/2, /8, /32, or internal RC)
 *   @ 20 MHz this gives Tad = 1.6 us, which meets the typical minimum.
 *   Use ADC_CLK_FRC if you need a
 *   fixed conversion clock independent of Fosc.
 * Result format: right-justified (10-bit in ADRESH:ADRESL)
 * Port config: all analog inputs enabled (AN0–AN7)
 * Acquisition delay loop count: 20 iterations (tune if needed)
 */

/* ADC clock selection (see ADC_Private.h for options) */
#define ADC_CLOCK_SEL       ADC_CLK_FOSC_32

/* ADCON1: port config + result justification */
#define ADC_PORT_CONFIG     ADC_ADCON1_ALL_ANALOG

/* Acquisition delay: number of NOP-equivalent loop iterations */
#define ADC_ACQUISITION_DLY  20U

#endif
