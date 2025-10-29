#!/bin/bash

# Build script for Serial Mock
# This script builds the serial mock application

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "Building Serial Mock..."
echo "Script directory: $SCRIPT_DIR"

# Create build directory if it doesn't exist
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
    echo "Created build directory: $BUILD_DIR"
fi

# Run CMake and build
cd "$BUILD_DIR"
cmake ..
make

echo "Build complete!"
echo "Binary location: $BUILD_DIR/serial_mock"
