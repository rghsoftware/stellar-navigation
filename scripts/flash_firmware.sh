#!/bin/bash
# Flash STM32 firmware

PLATFORM=${1:-stm32f405}

echo "📟 Flashing $PLATFORM..."
cd firmware/$PLATFORM
pio run --target upload
