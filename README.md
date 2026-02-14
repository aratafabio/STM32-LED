# STM32 LED Controller

STM32 project for controlling LEDs with animated effects, RGB LED and musical buzzer.

## Hardware

- **MCU:** STM32F4 (Discovery Board)
- **8 LEDs** driven via 74HC595 shift register
- **RGB LED** with software PWM
- **Passive buzzer** for melodies
- **UART** for terminal control (115200 baud)

### Passive Buzzer

| Buzzer    | Connection |
|-----------|------------|
| + (long)  | PE3        |
| - (short) | GND        |

### Full Wiring: 74HC595 + 8 LEDs

```
                    74HC595
                 ┌────┴────┐
         LED 1 ──┤ 1  Q1   VCC 16 ├── 3.3V
         LED 2 ──┤ 2  Q2   Q0  15 ├── LED 0
         LED 3 ──┤ 3  Q3   DS  14 ├── PE9  (STM32 Data)
         LED 4 ──┤ 4  Q4   OE  13 ├── GND
         LED 5 ──┤ 5  Q5   ST  12 ├── PE13 (STM32 Latch)
         LED 6 ──┤ 6  Q6   SH  11 ├── PE11 (STM32 Clock)
         LED 7 ──┤ 7  Q7   MR  10 ├── 3.3V
           GND ──┤ 8  GND  Q7'  9 ├── NC
                 └─────────────────┘
```

#### Control Signals (STM32 -> 74HC595)

| 74HC595 Pin | Function           | STM32 Pin |
|-------------|--------------------|-----------|
| DS (14)     | Serial Data        | PE9       |
| SHCP (11)   | Shift Clock        | PE11      |
| STCP (12)   | Storage Clock      | PE13      |

#### Power and Configuration

| 74HC595 Pin | Function           | Connection |
|-------------|--------------------|------------|
| VCC (16)    | Power Supply       | 3.3V       |
| GND (8)     | Ground             | GND        |
| MR (10)     | Master Reset       | 3.3V (always active) |
| OE (13)     | Output Enable      | GND (outputs always enabled) |
| Q7' (9)     | Serial Out         | NC (not connected) |

#### LED Outputs

| 74HC595 Pin | Output | Connection              |
|-------------|--------|-------------------------|
| 15          | Q0     | 220R Resistor -> LED 0 -> GND |
| 1           | Q1     | 220R Resistor -> LED 1 -> GND |
| 2           | Q2     | 220R Resistor -> LED 2 -> GND |
| 3           | Q3     | 220R Resistor -> LED 3 -> GND |
| 4           | Q4     | 220R Resistor -> LED 4 -> GND |
| 5           | Q5     | 220R Resistor -> LED 5 -> GND |
| 6           | Q6     | 220R Resistor -> LED 6 -> GND |
| 7           | Q7     | 220R Resistor -> LED 7 -> GND |

## Features

### LED Effects

| Effect | Description |
|--------|-------------|
| **Knight Rider** | Supercar-style chaser (default) |
| **Breath** | LEDs progressively turn on/off |
| **Manual Patterns** | 10 predefined patterns (0-9) |

### Buzzer Melodies

- Imperial March (Star Wars)
- It's a Long Way to the Top (AC/DC)
- Toccata and Fugue in D minor (J.S. Bach)

Press button B1 or send `P` via UART to play.

### SPI Loopback Test

Uses SPI5 (already configured for the onboard L3GD20 gyroscope) to verify SPI communication with a simple loopback.

```
PF9 (MOSI) ───jumper wire──── PF8 (MISO)

SCK = PF7 (internal, no wiring needed)
CS  = PC1 (disabled during test to avoid gyroscope interference)
```

Send `S` via UART to run the test. It transmits several byte patterns and checks that RX matches TX.

## UART Commands

| Command | Action |
|---------|--------|
| `A` | Automatic mode (Knight Rider) |
| `B` | Breath mode |
| `0-9` | Manual pattern |
| `P` | Play melody |
| `S` | SPI loopback test |
| `+` | Slow down animation |
| `-` | Speed up animation |

## Software Architecture

The project uses **FreeRTOS** with the following tasks:

```
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  DefaultTask    │  │  KnightRider    │  │  BreathTask     │
│  (UART + cmd)   │  │  (animation)    │  │  (animation)    │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                   │                    │
         └───────────────────┴────────────────────┘
                             │
                    ┌────────▼────────┐
                    │    shiftOut()   │
                    │   (74HC595)     │
                    └─────────────────┘

┌─────────────────┐
│   BuzzerTask    │ ◄── B1 Interrupt / 'P' Command
│   (melodies)    │
└─────────────────┘
```

## Build

Project generated with **STM32CubeMX**. Build with:

- STM32CubeIDE
- Makefile + arm-none-eabi-gcc
- PlatformIO

## File Structure

```
├── Core/
│   ├── Inc/          # Header files
│   └── Src/
│       └── main.c    # Main logic
├── Drivers/          # STM32 HAL
└── Middlewares/      # FreeRTOS, USB
```

## License

Personal code for educational purposes.
