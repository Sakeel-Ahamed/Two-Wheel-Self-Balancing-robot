# Two-Wheel Self-Balancing Robot 

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

```
📁 src/
├── main.cpp                # Main ESP32 (balancing, motor control, BLE)
├── main_ld06.cpp            # Lidar ESP32 (obstacle detection)
📁 visualisation/
├── index.html                # UI interface for local host
📁 include/
└── lib/
    └── (all supporting libraries)
```

## How to Run the Code

### 1. Main Controller (ESP32 #1)
Upload `main.cpp` to the ESP32 responsible for:
- Balancing control
- PID loop with motor actuation
- Receiving Bluetooth commands (via Dabble App)
- Obeying obstacle avoidance signals from LiDAR ESP32

### 2. LiDAR ESP32 (ESP32 #2)
Upload `main_ld06.cpp` to the second ESP32. This module:
- Processes data from the LD06 LiDAR sensor
- Sends real-time obstacle data to the main ESP32
- Avoids paths with obstacles within a configurable angle range

### 3. Web Interface
Run `index.html` on your local computer using:
```bash
python3 -m http.server 8000
```
Access the UI via `localhost:8000` to send commands or monitor robot status.

### 4. Bluetooth Control (Alternative)
Install the **Dabble app** on your smartphone and pair it with the robot's ESP32 to control via BLE buttons.

## Technical Details

### PID Controller

To maintain balance, the robot uses a PID loop:
```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```
Where:
- `e(t)` is the tilt angle error
- `Kp`, `Ki`, `Kd` are the tuned gain values
- The output `u(t)` controls motor PWM for corrective action

### Obstacle Avoidance Logic

The LD06 scans a 360° field and segments it by angle. If any object is detected within a critical distance (e.g., < 40cm) appropriate commands are sent to the main ESP32

The robot checks:
```cpp
if (distance[i] < threshold && angle[i]) {
    alertMainESP32();
}
```

## Known Issues / Future Improvements

- **Improved Control Algorithm:**  
  More robust balancing and smoother transitions between manual and autonomous modes are being worked on.

- **ROS Compatibility:**  
  Future iterations may use ROS 2 for sensor fusion, real-time plotting, and navigation stack integration.

## License

This project is developed for academic and learning purposes. Feel free to adapt and modify it for personal or educational use.
