#!/bin/bash
set -e

# build_bhy2cli_linux.sh
# Builds BHy2CLI for Linux PC

PROJECT_ROOT="$(dirname "$(dirname "$(readlink -f "$0")")")"
BHY2CLI_DIR="$PROJECT_ROOT/BHy2CLI"

echo ">>> Building BHy2CLI for Linux..."
echo "    Project Root: $PROJECT_ROOT"
echo "    BHy2CLI Dir:  $BHY2CLI_DIR"

cd "$BHY2CLI_DIR"

# Clean previous builds
echo ">>> Cleaning..."
make clean TARGET=PC

# Build for PC
echo ">>> Compiling..."
# Using -j$(nproc) for parallel build
make TARGET=PC -j$(nproc)

echo ">>> Build Complete!"

# Check if binary exists
if [ -f "bhy2cli" ]; then
    echo ">>> Binary created: bhy2cli"

    # Create release folder if properly
    mkdir -p release/Linux/bin
    mv bhy2cli release/Linux/bin/
    echo ">>> Moved binary to: release/Linux/bin/bhy2cli"
else
    echo "Error: Build failed, binary 'bhy2cli' not found."
    exit 1
fi
