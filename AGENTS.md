# ES_Course_Labs — Firmware Context

This repo is the firmware/driver repository for the autonomous car project.

## Current Platform

| Property | Value |
|---|---|
| MCU | PIC16F877A |
| Oscillator | 20 MHz HS |
| IDE/toolchain | MPLAB X + XC8 |
| Delay clock | `_XTAL_FREQ = 20000000UL` in `SERVICES/STD_TYPES.h` |
| Interrupt entry | `void __interrupt() isr(void)` in `MCAL/INTERRUPT_MANAGER/Interrupt_Manager.c` |
| Active app | `APP/main.c` calls `MANUAL_CONTROL_Test()` |

Older docs or comments that say MikroC, 8 MHz, or `void interrupt()` are stale unless the user explicitly asks to return to that setup.

## Build

The MPLAB project lives beside this repo at `../autonomous_car.X`.

From `../autonomous_car.X`, build with:

```powershell
& 'C:\Program Files\Microchip\MPLABX\v6.30\gnuBins\GnuWin32\bin\make.exe' -f nbproject\Makefile-default.mk SUBPROJECTS= .build-conf
```

The generated firmware is under `../autonomous_car.X/dist/default/production/`.

## Architecture Rules

Layering is:

```text
APP -> HAL -> MCAL -> SERVICES
```

- APP code must not access SFRs or raw register addresses.
- HAL code should call MCAL APIs and should not define MCU SFRs.
- MCAL owns register-level code.
- Keep the single XC8 interrupt entry in `MCAL/INTERRUPT_MANAGER/Interrupt_Manager.c`.
- Driver public APIs live in `*_Interface.h` / `*_interface.h`.
- Driver implementation files should include their own public interface first.

## Active Manual-Control Firmware

Primary files:

- `APP/main.c`
- `APP/manual_control.c`
- `APP/manual_control.h`
- `MCAL/USART/*`
- `MCAL/PWM/*`
- `HAL/MOTOR/*`
- `MCAL/GPIO/*`
- `MCAL/INTERRUPT_MANAGER/Interrupt_Manager.c`

Protocol:

- Pi -> PIC: `F`, `B`, `L`, `R`, `S` plus newline.
- PIC -> Pi: `BOOT\r\n`, `ACK:X\r\n`, `HB:N\r\n`.

Wiring:

- `RD0..RD3` -> L298N `IN1..IN4`.
- `RC2` -> L298N enable using CCP1 PWM.
- `RC6/TX`, `RC7/RX` -> Raspberry Pi UART.
- `RB0` -> heartbeat LED.

## Known Cleanup Backlog

Keep these clean during future edits:

1. Clock-dependent driver constants should match 20 MHz.
2. `EXT_INT_SetEdge()` follows PIC16F877A polarity: `INTEDG=1` rising, `0` falling.
3. HAL ultrasonic uses MCAL Timer1; do not reintroduce Timer1 SFRs in HAL.
4. UART RX foreground reads must stay interrupt-safe.

## Git

Commit only focused Codex changes in this repo. Do not push unless the user explicitly asks. Always avoid staging unrelated dirty files.
