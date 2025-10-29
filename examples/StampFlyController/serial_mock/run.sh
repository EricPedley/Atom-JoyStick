#!/bin/bash

# Run script for Serial Mock with virtual serial port
# This script sets up a virtual serial port pair and runs the mock

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="${SCRIPT_DIR}/build"
BINARY="${BUILD_DIR}/serial_mock"

# Check if binary exists
if [ ! -f "$BINARY" ]; then
    echo "Binary not found: $BINARY"
    echo "Please run ./build.sh first"
    exit 1
fi

# Check if socat is installed
if ! command -v socat &> /dev/null; then
    echo "socat is not installed. Please install it:"
    echo "  Ubuntu/Debian: sudo apt-get install socat"
    echo "  macOS: brew install socat"
    exit 1
fi

# Create virtual serial port pair
# socat creates two virtual serial ports that are connected to each other
echo "Creating virtual serial port pair..."
echo "  Port 1: /tmp/serial_mock_mcu"
echo "  Port 2: /tmp/serial_mock_host"

# Kill any existing socat processes for these ports
pkill -f "socat.*serial_mock" || true
sleep 1

# Create the virtual port pair in the background
socat -d -d PTY,link=/tmp/serial_mock_mcu,raw,echo=0 PTY,link=/tmp/serial_mock_host,raw,echo=0 &
SOCAT_PID=$!

# Give socat time to create the ports
sleep 2

# Check if ports were created
if [ ! -e /tmp/serial_mock_mcu ] || [ ! -e /tmp/serial_mock_host ]; then
    echo "Failed to create virtual serial ports"
    kill $SOCAT_PID || true
    exit 1
fi

echo "Virtual serial ports created successfully (socat PID: $SOCAT_PID)"
echo ""
echo "Starting Serial Mock (listening on /tmp/serial_mock_mcu)..."
echo "Use /tmp/serial_mock_host to send/receive data from your test scripts"
echo ""
echo "Cleanup on exit will terminate socat."
echo ""

# Trap to clean up on exit
cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $SOCAT_PID 2>/dev/null || true
    rm -f /tmp/serial_mock_mcu /tmp/serial_mock_host
    echo "Done!"
}

trap cleanup EXIT

# Run the serial mock
"$BINARY" /tmp/serial_mock_mcu
