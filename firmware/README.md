# Stellar Navigation Firmware

Embedded software for STM32 microcontrollers implementing star pattern recognition algorithms.

## Platforms

- **stm32f405/** - Black Pill (compact demo system)
- **stm32f767/** - NUCLEO (development platform with debugger)
- **shared/** - Common algorithms and protocol code

## Building

### Using PlatformIO
```bash
cd stm32f405
pio run
pio run --target upload
```

### Using STM32CubeIDE
Import project from respective directory.

## Protocol

See [../docs/protocol/uart_protocol.md](../docs/protocol/uart_protocol.md) for UART packet specification.
