# Serial Mock for StampFly Controller

This directory contains a serial mock for testing the StampFly controller's data sending and receiving without a real MCU (Microcontroller Unit). It's useful for:

- Testing serial communication protocols
- Debugging data format issues
- Validating packet structure and checksums
- Testing without physical hardware

## Overview

The serial mock consists of:

1. **serial_mock.cpp** - A simplified version of main.cpp containing only serial send/receive code
2. **CMakeLists.txt** - Build configuration
3. **build.sh** - Script to compile the mock
4. **run.sh** - Script to set up virtual serial ports and run the mock
5. **test_serial_mock.py** - Python test script to send data and verify responses

## Architecture

```
[test_serial_mock.py] <--> [/tmp/serial_mock_host] <--> (socat) <--> [/tmp/serial_mock_mcu] <--> [serial_mock binary]
  (Test Client)           (Virtual Port Pair via socat)            (Mock Server)
```

The setup uses `socat` to create a bidirectional virtual serial port pair:
- `/tmp/serial_mock_mcu` - Connected to the serial_mock application
- `/tmp/serial_mock_host` - Used by test scripts to send/receive data

## Prerequisites

### Linux (Ubuntu/Debian)
```bash
sudo apt-get install build-essential cmake socat python3-serial
```

### macOS
```bash
brew install cmake socat python3
pip3 install pyserial
```

## Building

```bash
cd serial_mock
./build.sh
```

This creates a `build/` directory with the compiled `serial_mock` binary.

## Running

### Terminal 1: Start the Mock Server
```bash
cd serial_mock
./run.sh
```

You should see output like:
```
Creating virtual serial port pair...
  Port 1: /tmp/serial_mock_mcu
  Port 2: /tmp/serial_mock_host
Virtual serial ports created successfully (socat PID: 12345)

Starting Serial Mock (listening on /tmp/serial_mock_mcu)...
Use /tmp/serial_mock_host to send/receive data from your test scripts
```

### Terminal 2: Run Tests
```bash
cd serial_mock
./test_serial_mock.py --packets 20 --delay 0.5
```

Or with custom settings:
```bash
./test_serial_mock.py --port /tmp/serial_mock_host --baudrate 115200 --packets 50 --delay 2.0
```

## Data Format

### Input (CSV from Test/Host)
The mock expects 10 comma-separated floats followed by a newline:

```
position[0],position[1],position[2],yaw,linear_velocity[0],linear_velocity[1],linear_velocity[2],positionSetpoint[0],positionSetpoint[1],positionSetpoint[2]
```

Example:
```
1.000,2.000,3.000,0.000,0.100,0.000,0.000,-1.000,-1.000,-1.000
```

Negative values in positionSetpoint trigger auto-calculation mode.

### Output (Binary Mocap Packet - 49 bytes)

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0-2 | 3 bytes | MAC | MAC address bytes 3-5 |
| 3-14 | 12 bytes | Position | 3 floats (x, y, z) |
| 15-18 | 4 bytes | Yaw | 1 float (yaw angle) |
| 19-30 | 12 bytes | Linear Velocity | 3 floats (vx, vy, vz) |
| 31-42 | 12 bytes | Position Target | 3 floats (target x, y, z) |
| 43 | 1 byte | Arm Button | Arm button state |
| 44 | 1 byte | Flip Button | Flip button state |
| 45 | 1 byte | Mode | Flight mode |
| 46 | 1 byte | Alt Mode | Altitude mode |
| 47 | 1 byte | Packet Counter | Sequential packet number |
| 48 | 1 byte | Checksum | Sum of bytes 0-47 |

**Total: 49 bytes**

## Usage Examples

### Example 1: Simple Continuous Test
```bash
./test_serial_mock.py --packets 100 --delay 0.1
```

This sends 100 packets with 100ms delay between each.

### Example 2: Single Packet Test
```bash
./test_serial_mock.py --packets 1
```

### Example 3: Custom Port
```bash
./test_serial_mock.py --port /dev/ttyUSB0 --packets 5
```

### Example 4: Using with Your Own Script

```python
import serial

ser = serial.Serial('/tmp/serial_mock_host', 115200, timeout=2.0)

# Send test data
ser.write(b'1.0,2.0,3.0,0.0,0.1,0.0,0.0,-1,-1,-1\n')

# Receive response (49 bytes)
response = ser.read(49)
print(f"Received {len(response)} bytes: {response.hex()}")

ser.close()
```

## Troubleshooting

### socat: Not found
```
Error: socat is not installed
```
**Solution:** Install socat using your package manager (see Prerequisites)

### Permission denied on serial ports
```
Permission denied: '/dev/ttyUSB0'
```
**Solution:** Add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Virtual ports not created
If run.sh says ports weren't created:
```bash
# Check if socat is running
ps aux | grep socat

# Manually create ports for debugging
socat -d -d PTY,link=/tmp/test1,raw,echo=0 PTY,link=/tmp/test2,raw,echo=0
```

### Checksum validation fails
Check that the mock is receiving the correct data format. The test script will print:
```
Warning: Checksum mismatch! Expected xx, got yy
```

## Protocol Details

### Packet Reception
1. Mock reads characters from serial until newline (`\n`)
2. Parses 10 floats from comma-separated values
3. Validates format - should have exactly 10 values

### Packet Transmission
1. Builds 49-byte binary mocap packet
2. Calculates checksum (sum of bytes 0-47)
3. Stores in last byte (index 48)
4. Sends complete packet via serial

### Checksum Calculation
```c
uint8_t sum = 0;
for (int i = 0; i < 48; i++) {
    sum += mocap_data[i];
}
mocap_data[48] = sum;
```

## Extending the Mock

To add more features:

1. **Auto-response mode** - Automatically respond with simulated telemetry
2. **Packet filtering** - Validate packet structure before responding
3. **Latency simulation** - Add configurable delays
4. **Error injection** - Introduce random packet corruption for robustness testing

## Performance Notes

- Recommended packet rate: 10-100 Hz (10-100 ms between packets)
- Binary packet size: 49 bytes
- Serial timeout: 2 seconds (configurable in test script)

## Integration with send_dummy_data.py

You can modify `send_dummy_data.py` to use `/tmp/serial_mock_host` instead of `/dev/ttyACM0`:

```python
serial_port = '/tmp/serial_mock_host'  # Or use environment variable
```

Then run both:
1. `./serial_mock/run.sh` in terminal 1
2. `python3 send_dummy_data.py` in terminal 2

## License

MIT License - See main LICENSE file in parent directory

## References

- socat manual: `man socat`
- pyserial documentation: https://pyserial.readthedocs.io/
- Original main.cpp structure and protocols
