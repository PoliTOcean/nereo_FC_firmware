# Project Structure

This document describes the main directories and files in the project.

## Root Directory

- `Core/` - Contains the main source code
  - `Inc/` - Header files
    - `navigation/` - Header files for navigation module (pwm computation and stabilization)
    - `FC_app.h` - Header file for main application
  - `Src/` - C/C++ Source files
    - `navigation/` - Source files for navigation module (pwm computation and stabilization)
    - `freertos.cpp` - Contains the main code (default task)

- `Debug/` - Contains binaries and object files.

- `Drivers/` - CMSIS and HAL drivers.
  
- `docs/` - Project documentation

- `micro_ros_stm32cubemx_utils/` - micro-ROS Library.

- `Middlewares/` - FreeRTOS and DSP libraries.
