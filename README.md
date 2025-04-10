# Two-Wheel Self-Balancing Robot with LiDAR Obstacle Detection and Autonomous Navigation

## Introduction

This project features a two-wheel self-balancing robot built on the ESP32 platform. The robot integrates multiple sensors including an MPU6050 gyroscope, LD06 LiDAR, and motor encoders to maintain balance, detect obstacles, and navigate autonomously in a user defined path.

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


---

## How to Run the Code

### 1. Main Controller (ESP32 #1)
Upload `main_controller.ino` to the ESP32 responsible for:
- Balancing control
- PID loop with motor actuation
- Receiving Bluetooth commands (via Dabble App)
- Obeying obstacle avoidance signals from LiDAR ESP32

### 2. LiDAR ESP32 (ESP32 #2)
Upload `lidar_node.ino` to the second ESP32. This module:
- Processes data from the LD06 LiDAR sensor
- Sends real-time obstacle data to the main ESP32
- Avoids paths with obstacles within a configurable angle range

### 3. Web Interface
Run `index.html` on your local computer using:
```bash
python3 -m http.server 8000

u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt

if (distance[i] < threshold && angle[i] within forward sector) {
    sendStopCommand();
}

