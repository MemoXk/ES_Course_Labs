#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

/*
 * SPI Configuration for PIC16F877A
 *
 * Mode: SPI Master
 * Clock speed: Fosc/16 = 1.25 MHz @ 20 MHz
 * Clock polarity (CPOL): 0 — SCK idle low
 * Clock phase   (CPHA): 0 — data sampled on rising edge, shifted on falling
 *   → SMP=0 (sample in middle), CKE=1 (transmit on active-to-idle = falling)
 *     matches SPI Mode 0,0 behaviour
 */

/* Speed selection (see SPI_Private.h) */
#define SPI_SPEED_MODE      SPI_MASTER_FOSC16

/* Clock polarity: 0 = idle-low, 1 = idle-high */
#define SPI_CKP_VALUE       0

/* Clock edge (CKE): 1 = transmit on active-to-idle edge */
#define SPI_CKE_VALUE       1

/* Sample point (SMP): 0 = middle of data output time */
#define SPI_SMP_VALUE       0

#endif
