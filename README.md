# DummyBot ROS2 - Autonomous Mobile Robot with Encoders

4WD autonomous mobile robot for industrial piece transportation using ROS2 Jazzy, computer vision, and AI technologies.

## 🤖 Hardware

- **Platform**: 4WD chassis with independent motor control
- **MCU**: ESP32 DevKit
- **Motor Drivers**: 2x L298N (4 DC motors)
- **Encoders**: 4x Quadrature encoders (1760 ticks/revolution)
- **Sensors**: 
  - LD06 LiDAR
  - Camera (for MediaPipe integration)
- **Computer**: Raspberry Pi 5 (16GB RAM)

## 🎯 Features

- ✅ Real-time odometry with wheel encoders
- ✅ PID velocity control on ESP32
- ✅ ROS2 Control integration (diff_drive_controller)
- ✅ Teleop keyboard control
- ✅ LiDAR-based navigation (SLAM capable)
- ✅ MediaPipe integration for gesture control
- ⚙️ Autonomous navigation (in development)

## 📋 Prerequisites

- Ubuntu 24.04 (Noble)
- ROS2 Jazzy
- Python 3.12+
- Arduino IDE or PlatformIO (for ESP32)

## 🚀 Quick Start

### 1. Clone and Build
```bash
git clone https://github.com/your-username/dummybot-ros2-encoder.git
cd dummybot-ros2-encoder

# Install dependencies
sudo apt update
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
                 ros-jazzy-teleop-twist-keyboard

# Build workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Flash ESP32 Firmware

See [firmware/esp32/README.md](firmware/esp32/README.md) for details.

### 3. Launch Robot
```bash
# Terminal 1 - Launch robot
ros2 launch dummybot_bringup dummybot.launch.py

# Terminal 2 - Bridge (for teleop compatibility)
python3 scripts/utils/cmd_vel_bridge.py

# Terminal 3 - Teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.3 -p turn:=15.0
```

## 📖 Documentation

- [Setup Guide](docs/setup/installation.md)
- [Hardware Specifications](docs/hardware/specifications.md)
- [Calibration Guide](docs/calibration/motor_calibration.md)
- [Troubleshooting](docs/troubleshooting/common_issues.md)

## 🎮 Controls

**Teleop Keyboard:**
- `i` - Forward
- `,` - Backward
- `j` - Rotate left
- `l` - Rotate right
- `k` - Stop
- `q/z` - Increase/decrease speed

## ⚙️ Current Configuration

- **Wheel separation**: 0.3863 m
- **Wheel radius**: 0.0325 m
- **Encoder resolution**: 1760 ticks/rev
- **PID rate**: 30 Hz
- **Max linear velocity**: 1.0 m/s
- **Max angular velocity**: 20.0 rad/s

## 🔧 Calibration

Current motor calibration factors (saved in ESP32 EEPROM):
- FL (Front Left): 1.0
- FR (Front Right): 1.67
- RL (Rear Left): 1.0
- RR (Rear Right): 1.0

## 📝 License

MIT License - see LICENSE file for details

## 👤 Author

Andrei - Politehnica București (UPB-FIIR)
Thesis Project 2025-2026
