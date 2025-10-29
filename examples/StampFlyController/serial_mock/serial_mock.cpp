/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack Technology CO LTD
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Serial Mock for StampFly Controller
 * 
 * This is a simplified version of main.cpp that only contains the serial
 * send/receive code for testing without a real MCU.
 * 
 * It simulates:
 * - Receiving telemetry data from the drone (mocap data format)
 * - Sending control commands to the drone
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// Mock serial port file descriptor
int serial_fd = -1;

// Buffer for serial communication
char serialBuffer[512];
int serialBufferPos = 0;
static constexpr uint8_t MOCAP_DATA_LEN = 3 + (3 + 1 + 3) * 4 + 12 + 1 + 1 + 3 + 1; // 49 bytes
static_assert(MOCAP_DATA_LEN == 49);

uint8_t mocap_data[MOCAP_DATA_LEN];
uint8_t senddata[MOCAP_DATA_LEN];
uint8_t packet_counter = 0;

// Mock control data
float Throttle = 0.0f;
float Phi = 0.0f;      // Roll
float Theta = 0.0f;    // Pitch
float Psi = 0.0f;      // Yaw
uint8_t auto_up_down_status = 0;
uint8_t flip_button = 0;
uint8_t mode = 0;
uint8_t alt_mode = 0;

// Mock peer MAC address
uint8_t peer_mac[6] = {0xF4, 0x12, 0xFA, 0x66, 0x80, 0x54};

/**
 * Initialize serial port in non-blocking mode
 */
int init_serial(const char *port, int baudrate) {
    serial_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd < 0) {
        std::cerr << "Failed to open serial port: " << port << std::endl;
        return -1;
    }

    struct termios tty;
    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "Error getting serial attributes" << std::endl;
        close(serial_fd);
        return -1;
    }

    // Set baud rate
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Set flags
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting serial attributes" << std::endl;
        close(serial_fd);
        return -1;
    }

    std::cout << "Serial port initialized: " << port << std::endl;
    return serial_fd;
}

/**
 * Close serial port
 */
void close_serial() {
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
}

/**
 * Send data to serial port
 */
int send_to_serial(const uint8_t *data, size_t len) {
    if (serial_fd < 0) return -1;
    return write(serial_fd, data, len);
}

/**
 * Read available data from serial port
 */
int read_from_serial(uint8_t *buffer, size_t max_len) {
    if (serial_fd < 0) return -1;
    return read(serial_fd, buffer, max_len);
}

/**
 * Process received mocap data
 * Expected format: position[0],position[1],position[2],yaw,linear_velocity[0],linear_velocity[1],linear_velocity[2],positionSetpoint[0],positionSetpoint[1],positionSetpoint[2]\n
 */
void process_mocap_data() {
    if (serialBufferPos == 0) return;

    float position[3] = {0, 0, 0}, yaw = 0, linear_velocity[3] = {0, 0, 0};
    float positionSetpoint[3] = {0, 0, 0};

    if (sscanf(serialBuffer, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
               &position[0], &position[1], &position[2], &yaw,
               &linear_velocity[0], &linear_velocity[1], &linear_velocity[2],
               &positionSetpoint[0], &positionSetpoint[1], &positionSetpoint[2]) == 10) {

        // Build mocap_data packet
        // a) first 3 bytes: our own MAC[3], MAC[4], MAC[5]
        mocap_data[0] = peer_mac[3];
        mocap_data[1] = peer_mac[4];
        mocap_data[2] = peer_mac[5];

        // b) position and yaw
        memcpy(&mocap_data[3], &position[0], sizeof(float));
        memcpy(&mocap_data[7], &position[1], sizeof(float));
        memcpy(&mocap_data[11], &position[2], sizeof(float));
        memcpy(&mocap_data[15], &yaw, sizeof(float));

        // c) linear velocity
        memcpy(&mocap_data[19], &linear_velocity[0], sizeof(float));
        memcpy(&mocap_data[23], &linear_velocity[1], sizeof(float));
        memcpy(&mocap_data[27], &linear_velocity[2], sizeof(float));

        // d) position target
        float positionTarget[3];
        if (positionSetpoint[2] < 0) {
            positionTarget[0] = position[0] + Theta;
            positionTarget[1] = position[1] + Phi;
            positionTarget[2] = position[2] + Throttle;
        } else {
            positionTarget[0] = positionSetpoint[0];
            positionTarget[1] = positionSetpoint[1];
            positionTarget[2] = positionSetpoint[2];
        }
        memcpy(&mocap_data[31], &positionTarget[0], sizeof(float));
        memcpy(&mocap_data[35], &positionTarget[1], sizeof(float));
        memcpy(&mocap_data[39], &positionTarget[2], sizeof(float));

        // e) buttons and mode
        mocap_data[43] = auto_up_down_status;
        mocap_data[44] = flip_button;
        mocap_data[45] = mode;
        mocap_data[46] = alt_mode;
        mocap_data[47] = packet_counter;
        packet_counter += 1;

        // f) checksum
        uint8_t sum = 0;
        for (int i = 0; i < MOCAP_DATA_LEN - 1; ++i) {
            sum += mocap_data[i];
        }
        mocap_data[MOCAP_DATA_LEN - 1] = sum;

        // Send the packet
        int bytes_sent = send_to_serial(mocap_data, MOCAP_DATA_LEN);
        if (bytes_sent > 0) {
            std::cout << "Sent mocap data (" << bytes_sent << " bytes): "
                      << "pos=(" << position[0] << "," << position[1] << "," << position[2] << ") "
                      << "yaw=" << yaw << " "
                      << "vel=(" << linear_velocity[0] << "," << linear_velocity[1] << "," << linear_velocity[2] << ")"
                      << std::endl;
        } else {
            std::cerr << "Failed to send mocap data" << std::endl;
        }

        serialBufferPos = 0;
        memset(serialBuffer, 0, sizeof(serialBuffer));
    } else {
        std::cerr << "Failed to parse CSV from serial. Expected: position[0],position[1],position[2],yaw,linear_velocity[0],linear_velocity[1],linear_velocity[2],arm_button" << std::endl;
        std::cerr << "Got: " << serialBuffer << std::endl;
        serialBufferPos = 0;
    }
}

/**
 * Main loop to receive and process serial data
 */
void serial_loop() {
    uint8_t buffer[256];
    int bytes_read;

    while (true) {
        bytes_read = read_from_serial(buffer, sizeof(buffer));
        if (bytes_read > 0) {
            for (int i = 0; i < bytes_read; i++) {
                char c = buffer[i];
                if (c != '\n' && serialBufferPos < sizeof(serialBuffer) - 1) {
                    serialBuffer[serialBufferPos++] = c;
                } else if (c == '\n') {
                    serialBuffer[serialBufferPos] = '\0';
                    if (serialBufferPos > 0) {
                        process_mocap_data();
                    }
                    serialBufferPos = 0;
                }
            }
        }

        // Small delay to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

/**
 * Main entry point
 */
int main(int argc, char *argv[]) {
    const char *port = "/dev/ttyUSB0";
    
    if (argc > 1) {
        port = argv[1];
    }

    std::cout << "StampFly Serial Mock - Starting" << std::endl;
    std::cout << "Using serial port: " << port << std::endl;
    std::cout << "Expected input format: position[0],position[1],position[2],yaw,linear_velocity[0],linear_velocity[1],linear_velocity[2],positionSetpoint[0],positionSetpoint[1],positionSetpoint[2]" << std::endl;

    if (init_serial(port, 115200) < 0) {
        std::cerr << "Failed to initialize serial port" << std::endl;
        return 1;
    }

    try {
        serial_loop();
    } catch (const std::exception &e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    close_serial();
    return 0;
}
