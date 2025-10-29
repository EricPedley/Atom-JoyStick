cmake_minimum_required(VERSION 3.10)
project(serial_mock)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the executable
add_executable(serial_mock serial_mock.cpp)

# Link pthread for threading support
target_link_libraries(serial_mock pthread)

# Set optimization flags
target_compile_options(serial_mock PRIVATE -Wall -Wextra -O2)
