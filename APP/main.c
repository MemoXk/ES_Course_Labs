/*
 * Device  : PIC16F877A @ 20 MHz HS oscillator
 * Compiler: MPLAB X + XC8
 *
 * Active test: UART manual motor control.
 *   Receives F/B/L/R/S plus U/D speed commands from Raspberry Pi
 *   (via /dev/serial0) and drives the L298N motors from 45-90% PWM.
 *   Echoes ACK after each command and sends a heartbeat counter.
 *
 * Wiring summary:
 *   RD0..RD3 -> L298N IN1..IN4
 *   RC2      -> L298N ENA + ENB  (jumper caps REMOVED)
 *   RC6 (TX) -> level shifter -> Pi pin 10 (GPIO15 RXD)
 *   RC7 (RX) -> level shifter -> Pi pin  8 (GPIO14 TXD)
 *   RB0      -> seat-belt switch input
 *   RD4      -> LDR digital input
 *   RD5      -> LDR indicator LED
 *   GND      -> Pi GND and L298N GND
 *
 * Pi side requires:
 *   /boot/firmware/config.txt: enable_uart=1, dtoverlay=disable-bt
 *   sudo systemctl disable hciuart && reboot
 *   app.py defaults to /dev/serial0; override with CAR_SERIAL_PORT if needed.
 */

// CONFIG
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config LVP  = OFF

#include "manual_control.h"

int main(void)
{
    MANUAL_CONTROL_Test();
    return 0;
}
