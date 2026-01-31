# STM32 LED Controller

Progetto STM32 per il controllo di LED con effetti animati, LED RGB e buzzer musicale.

## Hardware

- **MCU:** STM32F4 (Discovery Board)
- **8 LED** controllati via shift register 74HC595
- **LED RGB** con PWM software
- **Buzzer** passivo per melodie
- **UART** per controllo da terminale (115200 baud)

### Connessioni 74HC595

| Pin 74HC595 | Funzione | Pin STM32 |
|-------------|----------|-----------|
| DS (14)     | Data     | PE9       |
| SHCP (11)   | Clock    | PE11      |
| STCP (12)   | Latch    | PE13      |

## Funzionalità

### Effetti LED

| Effetto | Descrizione |
|---------|-------------|
| **Knight Rider** | Chenillard stile supercar (default) |
| **Breath** | LED si accendono/spengono progressivamente |
| **Pattern manuali** | 10 pattern predefiniti (0-9) |

### Melodie Buzzer

- Imperial March (Star Wars)
- It's a Long Way to the Top (AC/DC)
- Toccata e Fuga in Re minore (J.S. Bach)

Premere il pulsante B1 o inviare `P` via UART per suonare.

## Comandi UART

| Comando | Azione |
|---------|--------|
| `A` | Modalità automatica (Knight Rider) |
| `B` | Modalità Breath |
| `0-9` | Pattern manuale |
| `P` | Suona melodia |
| `+` | Rallenta animazione |
| `-` | Accelera animazione |

## Architettura Software

Il progetto usa **FreeRTOS** con i seguenti task:

```
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  DefaultTask    │  │  KnightRider    │  │  BreathTask     │
│  (UART + cmd)   │  │  (animazione)   │  │  (animazione)   │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                   │                    │
         └───────────────────┴────────────────────┘
                             │
                    ┌────────▼────────┐
                    │    shiftOut()   │
                    │   (74HC595)     │
                    └─────────────────┘

┌─────────────────┐
│   BuzzerTask    │ ◄── Interrupt B1 / Comando 'P'
│   (melodie)     │
└─────────────────┘
```

## Build

Progetto generato con **STM32CubeMX**. Compilare con:

- STM32CubeIDE
- Makefile + arm-none-eabi-gcc
- PlatformIO

## Struttura File

```
├── Core/
│   ├── Inc/          # Header files
│   └── Src/
│       └── main.c    # Logica principale
├── Drivers/          # HAL STM32
└── Middlewares/      # FreeRTOS, USB
```

## Licenza

Codice personale per scopi didattici.
