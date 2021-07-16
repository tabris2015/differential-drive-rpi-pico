# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/pepe/sources/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(diff_drive C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(${PROJECT_NAME} main.cpp src/servo.cpp src/dc_motor.cpp src/encoder.cpp src/pid_controller.cpp src/robot.cpp include/robot.h include/robot_pins.h)

pico_set_program_name(${PROJECT_NAME} "diff_drive")
pico_set_program_version(${PROJECT_NAME} "0.1")

target_include_directories(${PROJECT_NAME} PRIVATE include)

# Add the standard library to the build
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_adc)

# Add any user requested libraries
target_link_libraries(${PROJECT_NAME}
        hardware_timer
        hardware_pwm
        )

pico_add_extra_outputs(${PROJECT_NAME})

# Enable USB serial
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

