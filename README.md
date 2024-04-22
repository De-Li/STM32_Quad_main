# STM32_Quad_main

This repository contains the source code for a multicopter control firmware on STM32 board (we used STM32MP157D-DK1). The firmware is designed to run on a custom-built multicopter and provides the necessary control algorithms to stabilize and maneuver the quadcopter in flight.

## Overview

The multicopter control firmware is implemented in C/C++ and is designed to run on STM32-series board. It includes modules for:

- Sensor data acquisition and fusion
- PID flight control algorithms
- Motor control and PWM fusion and generation
- FUTABA telemetry communication interfaces for remote control via UART DMA transfer
- Uart-based(DMA) matlab simulation

## Features

- **Stabilization:** Implements algorithms for stabilizing the multicopter by adjusting motor speeds based on sensor readings (e.g., gyroscopes, accelerometers).
- **Flight Modes:** Supports two flight modes: stabilized and acro mode.
- **PID Control:** Utilizes Proportional-Integral-Derivative (PID) control loops for precise control of the quadcopter's attitude and position.
- **Telemetry:** Provides telemetry data transmission to a FUTABA remote controller for monitoring flight parameters and diagnostics.
- **Configurability:** Allows for parameter tuning.

## Getting Started

To build and run the quadcopter control firmware, follow these steps:

1. **Hardware Setup:** Assemble the quadcopter hardware components including motors, ESCs (Electronic Speed Controllers), flight controller board, and sensors (gyroscope, accelerometer, etc.).
3. **Software Setup:** If do simulation, download Ardupilot's Matlab simulink to simulate.
4. **Software:** Clone this repository to your development environment and install STM32 IDE.
5. **Build Firmware:** import repository into STM32 IDE to compile the firmware.
6. **Upload Firmware:** Upload the compiled firmware to the flight controller board using STM32 IDE tools.
7. **Calibration:** Calibrate the sensors and tune the PID parameters for optimal performance.
8. **Flight Testing:** Conduct initial flight tests in a controlled environment to verify the stability and performance of the multicopter.

Test video: https://www.youtube.com/watch?v=HgrJxudFslQ
