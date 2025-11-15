#!/bin/bash
# Build entire stellar navigation system

set -e
echo "🚀 Building Stellar Navigation System"

# Build firmware
echo "📟 Building firmware..."
cd firmware/stm32f405
pio run || echo "⚠️  PlatformIO not found, skipping firmware build"
cd ../..

# Build cFS
echo "🛰️  Building cFS..."
if [ -d ~/workspace/cFS ]; then
    cd ~/workspace/cFS
    make distclean
    make prep
    make -j$(nproc)
    cd -
else
    echo "⚠️  cFS not found at ~/workspace/cFS, skipping"
fi

# Python dashboard
echo "🐍 Setting up Python environment..."
cd dashboard
if command -v uv &> /dev/null; then
    uv sync
else
    echo "❌ uv not found! Install with: curl -LsSf https://astral.sh/uv/install.sh | sh"
    exit 1
fi
cd ..

echo "✅ Build complete!"
