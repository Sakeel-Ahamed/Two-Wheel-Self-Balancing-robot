# Two-Wheel Self-Balancing Robot with LiDAR Obstacle Detection and Autonomous Navigation

## Introduction

This project features a two-wheel self-balancing robot built on the ESP32 platform. The robot integrates multiple sensors including an MPU6050 gyroscope, LD06 LiDAR, and motor encoders to maintain balance, detect obstacles, and navigate autonomously in a defined environment.

The system includes two ESP32 modules: one for core control logic (motor actuation, PID balancing, BLE interface) and another for LiDAR processing and obstacle detection. The robot supports both manual Bluetooth-based control and autonomous path-following toward given coordinates.

## User Installation Instructions

### Prerequisites

1. **PlatformIO**  
   Install the [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) in VS Code.

2. **Board Initialization**
   - Open the project folder in PlatformIO.
   - Set the board as `esp32dev` in `platformio.ini`.

3. **Required Libraries**
   Make sure the following libraries are installed (either via PlatformIO or manually):
   - `ESP32Servo`
   - `ESPAsyncWebServer`
   - `I2Cdev`
   - `MPU6050`
   - `LD06`
   - `Teleplot`

You may install them via `platformio.ini` or the PlatformIO library manager.

### File Structure

