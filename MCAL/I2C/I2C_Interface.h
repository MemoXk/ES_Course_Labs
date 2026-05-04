#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include "../../SERVICES/STD_TYPES.h"
#include "../../SERVICES/BIT_MATH.h"
#include "../GPIO/GPIO_interface.h"
#include "I2C_Private.h"
#include "I2C_Config.h"

/* ACK/NACK constants for I2C_ReadByte() */
#define I2C_SEND_ACK    1
#define I2C_SEND_NACK   0

/*
 * Initialize MSSP in I2C master mode.
 * Configures RC3 (SCL) and RC4 (SDA) as inputs (open-drain).
 * Sets baud rate from I2C_Config.h.
 */
void I2C_MasterInit(void);

/* Generate I2C START condition — blocks until complete */
void I2C_Start(void);

/* Generate I2C STOP condition — blocks until complete */
void I2C_Stop(void);

/* Generate I2C REPEATED START condition — blocks until complete */
void I2C_RepeatedStart(void);

/*
 * Write one byte to the bus (address byte or data byte).
 * Returns: 0 if slave ACKed, 1 if slave NACKed.
 */
u8 I2C_WriteByte(u8 Data);

/*
 * Read one byte from the bus.
 * ack: I2C_SEND_ACK (1) to continue reading, I2C_SEND_NACK (0) for last byte.
 * Returns the received byte.
 */
u8 I2C_ReadByte(u8 ack);

/* Disable the MSSP/I2C module */
void I2C_Disable(void);

#endif
