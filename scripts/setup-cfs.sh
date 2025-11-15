#!/bin/bash
# Setup cFS submodule and create symlinks to mission configuration
# Run this after cloning the repository

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CFS_DIR="$REPO_ROOT/cfs"
MISSION_DIR="$REPO_ROOT/cfs-mission"

echo "========================================"
echo "cFS Setup Script"
echo "========================================"
echo ""

# Check if we're in the right place
if [ ! -d "$MISSION_DIR" ]; then
    echo "❌ Error: cfs-mission/ directory not found"
    echo "   Are you running this from the stellar-navigation repository?"
    exit 1
fi

# Initialize cFS submodule if not already done
if [ ! -d "$CFS_DIR/cfe" ]; then
    echo "📥 Initializing cFS submodule..."
    cd "$REPO_ROOT"
    git submodule update --init --recursive
    echo "✅ cFS submodule initialized"
else
    echo "✅ cFS submodule already initialized"
fi

echo ""
echo "🔗 Creating symlinks..."

# Create symlink for Makefile
if [ -L "$CFS_DIR/Makefile" ]; then
    echo "   ℹ️  Makefile symlink already exists"
elif [ -f "$CFS_DIR/Makefile" ]; then
    echo "   ⚠️  Warning: cfs/Makefile exists but is not a symlink"
    read -p "   Replace with symlink? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm "$CFS_DIR/Makefile"
        ln -s ../cfs-mission/Makefile "$CFS_DIR/Makefile"
        echo "   ✅ Created Makefile symlink"
    fi
else
    ln -s ../cfs-mission/Makefile "$CFS_DIR/Makefile"
    echo "   ✅ Created Makefile symlink"
fi

# Create symlink for sample_defs
if [ -L "$CFS_DIR/sample_defs" ]; then
    echo "   ℹ️  sample_defs symlink already exists"
elif [ -d "$CFS_DIR/sample_defs" ]; then
    echo "   ⚠️  Warning: cfs/sample_defs exists but is not a symlink"
    read -p "   Replace with symlink? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$CFS_DIR/sample_defs"
        ln -s ../cfs-mission/sample_defs "$CFS_DIR/sample_defs"
        echo "   ✅ Created sample_defs symlink"
    fi
else
    ln -s ../cfs-mission/sample_defs "$CFS_DIR/sample_defs"
    echo "   ✅ Created sample_defs symlink"
fi

# Create apps directory if it doesn't exist
if [ ! -d "$CFS_DIR/apps" ]; then
    mkdir -p "$CFS_DIR/apps"
    echo "   ✅ Created cfs/apps directory"
fi

# Create symlink for starnav app
if [ -L "$CFS_DIR/apps/starnav" ]; then
    echo "   ℹ️  starnav app symlink already exists"
elif [ -d "$CFS_DIR/apps/starnav" ]; then
    echo "   ⚠️  Warning: cfs/apps/starnav exists but is not a symlink"
    read -p "   Replace with symlink? (y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$CFS_DIR/apps/starnav"
        ln -s ../../cfs-mission/apps/starnav "$CFS_DIR/apps/starnav"
        echo "   ✅ Created starnav app symlink"
    fi
else
    ln -s ../../cfs-mission/apps/starnav "$CFS_DIR/apps/starnav"
    echo "   ✅ Created starnav app symlink"
fi

echo ""
echo "========================================"
echo "✅ Setup complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "  1. Build cFS:"
echo "     cd cfs"
echo "     make SIMULATION=native prep"
echo "     make -j\$(nproc)"
echo "     make install"
echo ""
echo "  2. Run cFS:"
echo "     cd cfs/build/exe/cpu1"
echo "     ./core-cpu1"
echo ""
