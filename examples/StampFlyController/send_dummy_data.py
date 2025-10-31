import serial
import time
import datetime
import sys
from pathlib import Path

from pyvicon_datastream import tools
import numpy as np

VICON_TRACKER_IP = "192.168.30.153"
OBJECT_NAME = "stampfly"

# cam = USBCam(None, USBCam.ResolutionOption.R1080P, video_path="/dev/video0", framerate=30)

# while True:
#     frame = cam.take_image()
#     if frame is not None:
#         cv2.imshow("Camera", frame.get_array())
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

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
    serial_port = '/dev/ttyACM1'
    baud_rate = 115200  # Common baud rate, adjust if needed
    timeout = 1.0  # 1 second timeout
    
    print(f"Starting data logger for {serial_port}")
    print("Press Ctrl+C to exit")
    
    current_csv_file = None
    current_csv_writer = None

    mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
    
    while True:
        try:
            # Try to open serial connection
            print(f"Attempting to connect to {serial_port}...")
            with serial.Serial(serial_port, baud_rate, timeout=timeout) as ser:
                print(f"Connected to {serial_port} at {baud_rate} baud")
                
                while True:
                    try:
                        position_data = mytracker.get_position(OBJECT_NAME)
                        _, frame_no, objects = position_data
                        if len(objects)>0:
                            sixdof_pose = np.array(objects[0][2:])
                            print(f"Position: {sixdof_pose}")
                        else:
                            print("No objects")
                                            # Read a line from serial port
                        position = sixdof_pose[:3]/1e3
                        yaw = sixdof_pose[5]
                        linear_velocity = [0,0,0]
                        positionSetpoint = [-1.5, 2, 0.5]
                        to_send = "{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}\n".format(
                            position[0], position[1], position[2],
                            yaw,
                            linear_velocity[0], linear_velocity[1], linear_velocity[2],
                            positionSetpoint[0], positionSetpoint[1], positionSetpoint[2]
                        )
                        print(f"Sending: {to_send.strip()}")
                        ser.write(to_send.encode('utf-8'))
                        line = ser.readline().decode('utf-8').strip()
                        
                        if line:

                            print(f"(Python) Received from ESP: {line}")
                        
                        else:
                            # No data received within timeout
                            if current_csv_file is not None:
                                print("No data received for 1 second, closing current CSV file")
                                # cam.stop_recording()
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