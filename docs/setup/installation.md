# DummyBot Installation Guide

Complete setup guide for DummyBot ROS2 system on Raspberry Pi 5.

## System Requirements

### Hardware
- Raspberry Pi 5 (16GB RAM recommended)
- MicroSD card (32GB+ recommended)
- ESP32 DevKit
- USB cable for ESP32
- 7-12V power supply for motors

### Software
- Ubuntu 24.04 Noble (arm64)
- ROS2 Jazzy Jalisco
- Python 3.12+

## Installation Steps

### 1. Ubuntu and ROS2 Installation
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Jazzy (if not already installed)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

### 2. Install ROS2 Control Dependencies
```bash
sudo apt install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-xacro
```

### 3. Clone and Build Workspace
```bash
# Clone repository
cd ~
git clone https://github.com/LicentaFIIR-2026/dummybot-ros2-encoder.git
cd dummybot-ros2-encoder

# Source ROS2
source /opt/ros/jazzy/setup.bash

# **IMPORTANT: Clone ros2_control in workspace to avoid ABI issues**
cd ros2_ws/src
git clone https://github.com/ros-controls/ros2_control.git -b jazzy
git clone https://github.com/ros-controls/ros2_controllers.git -b jazzy

# Build only essential packages
cd ~/dummybot-ros2-encoder/ros2_ws
colcon build --symlink-install \
  --packages-select dummybot_bringup dummybot_control \
  dummybot_description dummybot_navigation \
  controller_manager controller_interface hardware_interface \
  diff_drive_controller joint_state_broadcaster

# Source workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/dummybot-ros2-encoder/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Note:** You may see warnings about missing packages (e.g., `state_interfaces_broadcaster`). These are normal and can be ignored - they don't affect robot operation.

### 4. Install Additional Dependencies
```bash
# Install Python dependencies for bridge scripts
pip3 install --user rclpy geometry-msgs

# Install serial permissions
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

### 5. Flash ESP32 Firmware

See [firmware/esp32/README.md](../../firmware/esp32/README.md) for detailed instructions.

**Quick start:**
```bash
# Install Arduino IDE or PlatformIO
# Open firmware/esp32/DummyBot_ESP32_Bridge.ino
# Select Board: ESP32 Dev Module
# Upload to ESP32
```

### 6. Verify Installation
```bash
# Check ROS2 installation
ros2 --version

# Check installed packages
ros2 pkg list | grep dummybot

# Test serial connection
ls -l /dev/ttyUSB*
```

## Post-Installation Configuration

### 1. Set Serial Port Permissions
```bash
# Add udev rule for persistent permissions
sudo tee /etc/udev/rules.d/99-esp32.rules << 'RULE'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666"
RULE

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. Configure Network (Optional)

For remote operation:
```bash
# Set static IP on Raspberry Pi
sudo nano /etc/netplan/01-netcfg.yaml
```

### 3. Test ESP32 Communication
```bash
# Open serial monitor
screen /dev/ttyUSB0 115200

# Test commands
> e
0 0 0 0
> b
115200
```

Press `Ctrl+A` then `K` then `Y` to exit screen.

## Troubleshooting

### ROS2 build fails
```bash
# Clean and rebuild
cd ~/dummybot-ros2-encoder/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Serial port not found
```bash
# Check USB devices
lsusb
ls -l /dev/ttyUSB*

# Check permissions
groups $USER
# Should include 'dialout'
```

### Import errors
```bash
# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/dummybot-ros2-encoder/ros2_ws/install/setup.bash
```

## Next Steps

- [Hardware Specifications](../hardware/specifications.md)
- [Motor Calibration Guide](../calibration/motor_calibration.md)
- [Quick Start Guide](../../README.md#quick-start)
