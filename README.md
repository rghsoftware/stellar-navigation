# Stellar Navigation System

Autonomous spacecraft attitude determination using star pattern recognition.

## Components

- **firmware/** - STM32 embedded software (navigation algorithms)
- **cfs-apps/** - Core Flight System applications (STARNAV_APP)
- **dashboard/** - Python web interface with 3D visualization
- **test-data/** - Synthetic star field observations
- **docs/** - Comprehensive documentation
- **scripts/** - Build automation and utilities
- **tools/** - Development and analysis tools

## Quick Start

See [docs/setup/README.md](docs/setup/README.md) for setup instructions.

## Architecture

```
STM32 (Algorithms) → UART → Pi + cFS → WebSocket → Browser Dashboard
                                    ↓
                                  Servos (Physical Gimbal)
```

## Documentation

- [Pi 5 Setup Guide](docs/setup/pi5_setup.md)
- [STM32 Firmware Setup](docs/setup/stm32_setup.md)
- [System Architecture](docs/architecture/system_design.md)
- [UART Protocol Specification](docs/protocol/uart_protocol.md)

## Building

```bash
# Build everything
./scripts/build_all.sh

# Build individual components
cd firmware/stm32f405 && pio run
cd cfs-apps && ./link_to_cfs.sh && cd ~/workspace/cFS && make
cd dashboard && pip install -r requirements.txt
```

## License

[Your chosen license]
