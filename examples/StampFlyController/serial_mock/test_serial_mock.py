#!/usr/bin/env python3

"""
Test script for Serial Mock
Sends dummy mocap data to the serial mock and displays received packets
"""

import serial
import time
import struct
import sys
import argparse
from pathlib import Path

def create_mocap_packet(position, yaw, linear_velocity, position_setpoint=None):
    """
    Create a mocap data packet in the expected CSV format
    
    Format: position[0],position[1],position[2],yaw,linear_velocity[0],linear_velocity[1],linear_velocity[2],positionSetpoint[0],positionSetpoint[1],positionSetpoint[2]\n
    """
    if position_setpoint is None:
        position_setpoint = [-1, -1, -1]  # Negative indicates auto-calculation
    
    line = "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
        position[0], position[1], position[2],
        yaw,
        linear_velocity[0], linear_velocity[1], linear_velocity[2],
        position_setpoint[0], position_setpoint[1], position_setpoint[2]
    )
    return line

def parse_mocap_response(data):
    """
    Parse a mocap response packet (49 bytes)
    
    Layout:
    - 0-2: MAC address (3 bytes)
    - 3-14: Position (3 floats, 12 bytes)
    - 15-18: Yaw (1 float, 4 bytes)
    - 19-30: Linear velocity (3 floats, 12 bytes)
    - 31-42: Position target (3 floats, 12 bytes)
    - 43: Arm button (1 byte)
    - 44: Flip button (1 byte)
    - 45: Mode (1 byte)
    - 46: Alt mode (1 byte)
    - 47: Packet counter (1 byte)
    - 48: Checksum (1 byte)
    """
    if len(data) != 49:
        return None
    
    mac = tuple(data[0:3])
    position = struct.unpack('<fff', data[3:15])
    yaw = struct.unpack('<f', data[15:19])[0]
    velocity = struct.unpack('<fff', data[19:31])
    position_target = struct.unpack('<fff', data[31:43])
    arm_button = data[43]
    flip_button = data[44]
    mode = data[45]
    alt_mode = data[46]
    packet_counter = data[47]
    checksum = data[48]
    
    # Verify checksum
    calculated_sum = sum(data[0:48]) & 0xFF
    if calculated_sum != checksum:
        print(f"Warning: Checksum mismatch! Expected {checksum:02x}, got {calculated_sum:02x}")
    
    return {
        'mac': mac,
        'position': position,
        'yaw': yaw,
        'velocity': velocity,
        'position_target': position_target,
        'arm_button': arm_button,
        'flip_button': flip_button,
        'mode': mode,
        'alt_mode': alt_mode,
        'packet_counter': packet_counter,
        'checksum': checksum,
        'checksum_valid': calculated_sum == checksum
    }

def test_serial_mock(port, baudrate=115200, num_packets=10, delay=1.0):
    """
    Send test data to serial mock and receive responses
    """
    print(f"Connecting to serial port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Test packets: {num_packets}")
    print()
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2.0)
        print(f"Connected! Starting test...")
        print()
        
        time.sleep(0.5)  # Give the mock time to initialize
        
        for packet_num in range(num_packets):
            # Create test data with increasing position
            position = [1.0 + packet_num * 0.1, 2.0, 3.0]
            yaw = 0.0
            linear_velocity = [0.1, 0.0, 0.0]
            
            # Send mocap data
            data_str = create_mocap_packet(position, yaw, linear_velocity)
            print(f"[{packet_num:02d}] Sending: {data_str.strip()}")
            
            ser.write(data_str.encode('utf-8'))
            time.sleep(0.1)
            
            # Try to receive response
            try:
                response_data = ser.read(49)
                if len(response_data) == 49:
                    response = parse_mocap_response(response_data)
                    if response:
                        print(f"[{packet_num:02d}] Received mocap packet:")
                        print(f"       MAC: {response['mac']}")
                        print(f"       Position: ({response['position'][0]:.3f}, {response['position'][1]:.3f}, {response['position'][2]:.3f})")
                        print(f"       Yaw: {response['yaw']:.3f}")
                        print(f"       Velocity: ({response['velocity'][0]:.3f}, {response['velocity'][1]:.3f}, {response['velocity'][2]:.3f})")
                        print(f"       Position Target: ({response['position_target'][0]:.3f}, {response['position_target'][1]:.3f}, {response['position_target'][2]:.3f})")
                        print(f"       Arm: {response['arm_button']}, Flip: {response['flip_button']}, Mode: {response['mode']}")
                        print(f"       Packet #: {response['packet_counter']}, Checksum: {'✓' if response['checksum_valid'] else '✗'}")
                    else:
                        print(f"[{packet_num:02d}] Failed to parse response")
                elif len(response_data) > 0:
                    print(f"[{packet_num:02d}] Received {len(response_data)} bytes (expected 49)")
                    print(f"       Data (hex): {response_data.hex()}")
                else:
                    print(f"[{packet_num:02d}] No response received (timeout)")
            except serial.SerialTimeoutException:
                print(f"[{packet_num:02d}] Timeout waiting for response")
            
            print()
            time.sleep(delay)
        
        ser.close()
        print("Test complete!")
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        if ser.is_open:
            ser.close()
        sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description='Test Serial Mock')
    parser.add_argument('--port', default='/tmp/serial_mock_host',
                        help='Serial port to connect to (default: /tmp/serial_mock_host)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('--packets', type=int, default=10,
                        help='Number of test packets to send (default: 10)')
    parser.add_argument('--delay', type=float, default=1.0,
                        help='Delay between packets in seconds (default: 1.0)')
    
    args = parser.parse_args()
    
    test_serial_mock(args.port, args.baudrate, args.packets, args.delay)

if __name__ == '__main__':
    main()
