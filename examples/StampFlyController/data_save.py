import serial
import csv
import time
import datetime
import os
import sys
from pathlib import Path

# Data format from the device:
# USBSerial.printf("%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
#                  elapsedTime, accelX, accelY, accelZ, roll_rate, pitch_rate, yaw_rate,
#                  frontRight_motor_duty, frontLeft_motor_duty, rearRight_motor_duty, rearLeft_motor_duty);

# CSV column headers
CSV_HEADERS = [
    'elapsedTime', 'accelX', 'accelY', 'accelZ', 
    'roll_rate', 'pitch_rate', 'yaw_rate',
    'frontRight_motor_duty', 'frontLeft_motor_duty', 
    'rearRight_motor_duty', 'rearLeft_motor_duty'
]

def create_timestamped_filename():
    """Create a CSV filename based on current timestamp"""
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = Path("flight_data")
    folder.mkdir(parents=True, exist_ok=True)
    return f"{folder}/{timestamp}.csv"

def read_and_save_data():
    """Main function to read data from serial port and save to CSV files"""
    serial_port = '/dev/ttyACM0'
    baud_rate = 115200  # Common baud rate, adjust if needed
    timeout = 1.0  # 1 second timeout
    
    print(f"Starting data logger for {serial_port}")
    print("Press Ctrl+C to exit")
    
    current_csv_file = None
    current_csv_writer = None
    
    while True:
        try:
            # Try to open serial connection
            print(f"Attempting to connect to {serial_port}...")
            with serial.Serial(serial_port, baud_rate, timeout=timeout) as ser:
                print(f"Connected to {serial_port} at {baud_rate} baud")
                
                while True:
                    try:
                        # Read a line from serial port
                        line = ser.readline().decode('utf-8').strip()
                        
                        if line:
                            # If we have a line of data
                            print(f"Received: {line}")
                            
                            # If no CSV file is open, create a new one
                            if current_csv_file is None:
                                filename = create_timestamped_filename()
                                print(f"Creating new CSV file: {filename}")
                                current_csv_file = open(filename, 'w', newline='')
                                current_csv_writer = csv.writer(current_csv_file)
                                # Write headers
                                current_csv_writer.writerow(CSV_HEADERS)
                                current_csv_file.flush()
                            
                            # Parse and write the data
                            try:
                                # Split the comma-separated values
                                values = line.split(',')
                                if len(values) == 11:  # Expected number of values
                                    # Convert to appropriate types
                                    parsed_values = []
                                    # First 7 values are floats
                                    for i in range(7):
                                        parsed_values.append(float(values[i]))
                                    # Last 4 values are integers
                                    for i in range(7, 11):
                                        parsed_values.append(int(values[i]))
                                    
                                    # Write to CSV
                                    current_csv_writer.writerow(parsed_values)
                                    current_csv_file.flush()
                                else:
                                    print(f"Warning: Expected 11 values, got {len(values)}: {line}")
                            except (ValueError, IndexError) as e:
                                print(f"Error parsing line '{line}': {e}")
                        
                        else:
                            # No data received within timeout
                            if current_csv_file is not None:
                                print("No data received for 1 second, closing current CSV file")
                                current_csv_file.close()
                                current_csv_file = None
                                current_csv_writer = None
                    
                    except serial.SerialTimeoutException:
                        # Timeout occurred
                        if current_csv_file is not None:
                            print("Serial timeout, closing current CSV file")
                            current_csv_file.close()
                            current_csv_file = None
                            current_csv_writer = None
                    
                    except UnicodeDecodeError as e:
                        print(f"Unicode decode error: {e}")
                        continue
        
        except serial.SerialException as e:
            print(f"Serial port error: {e}")
            if current_csv_file is not None:
                current_csv_file.close()
                current_csv_file = None
                current_csv_writer = None
            print("Waiting 2 seconds before retrying...")
            time.sleep(2)
        
        except KeyboardInterrupt:
            print("\nShutdown requested by user")
            if current_csv_file is not None:
                current_csv_file.close()
            sys.exit(0)
        
        except Exception as e:
            print(f"Unexpected error: {e}")
            if current_csv_file is not None:
                current_csv_file.close()
                current_csv_file = None
                current_csv_writer = None
            time.sleep(1)

if __name__ == "__main__":
    read_and_save_data()