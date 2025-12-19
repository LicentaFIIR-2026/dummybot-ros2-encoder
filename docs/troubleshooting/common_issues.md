# Troubleshooting Guide

Common issues and solutions for DummyBot.

## Table of Contents

1. [Hardware Issues](#hardware-issues)
2. [Communication Issues](#communication-issues)
3. [Motor Control Issues](#motor-control-issues)
4. [ROS2 Issues](#ros2-issues)
5. [Navigation Issues](#navigation-issues)

---

## Hardware Issues

### ESP32 Not Detected

**Symptoms:**
- `/dev/ttyUSB0` not found
- `ls /dev/ttyUSB*` returns nothing

**Solutions:**

1. **Check USB connection:**
```bash
lsusb
# Should show: "QinHeng Electronics HL-340 USB-Serial adapter" or similar
```

2. **Check cable:**
- Try different USB cable (data cable, not charge-only)
- Try different USB port on Raspberry Pi

3. **Add to dialout group:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
groups
# Should include 'dialout'
```

4. **Load USB driver:**
```bash
sudo modprobe ch341
dmesg | tail
# Should show ttyUSB device
```

### Motors Don't Move

**Symptoms:**
- Commands sent but no motor response
- L298N LEDs may be on/off

**Solutions:**

1. **Check power supply:**
```bash
# Measure battery voltage (should be 7-12V)
# Check L298N power LED
# Verify ESP32 has power (LED on)
```

2. **Test with raw PWM:**
```bash
screen /dev/ttyUSB0 115200
> o 200 200 200 200
# All motors should run
```

3. **Check L298N enable jumpers:**
- ENA and ENB jumpers must be IN PLACE
- Remove jumpers if using PWM control

4. **Verify wiring:**
```bash
# Check pinout against config.h
cat ~/dummybot-ros2-encoder/firmware/esp32/config.h | grep "define.*_L\|define.*_R"
```

5. **Check motor drivers:**
- L298N may overheat (add heatsinks)
- Test with different L298N if available
- Check for short circuits

### Encoders Not Reading

**Symptoms:**
- `> e` returns all zeros
- Odometry doesn't update
- Robot moves but no encoder feedback

**Solutions:**

1. **Test encoder reading:**
```bash
screen /dev/ttyUSB0 115200
> r  # Reset encoders
> e  # Should show: 0 0 0 0
# Manually rotate wheels
> e  # Values should change
```

2. **Check encoder power:**
- Encoders need 5V (not 3.3V)
- Verify VCC and GND connections
- Check continuity with multimeter

3. **Verify encoder wiring:**
```bash
# Check pinout
cat ~/dummybot-ros2-encoder/firmware/esp32/config.h | grep ENCODER
```

4. **Test individual encoders:**
```bash
# Rotate each wheel and check encoder values
> e
FL FR RL RR
# Only one value should change per wheel
```

5. **Check for noise:**
- Keep encoder wires away from motor power wires
- Use twisted pairs for A/B channels
- Add pull-up resistors (10kΩ) if needed

### One Motor Slower/Faster

**Symptoms:**
- Robot veers during straight motion
- Uneven rotation

**Solutions:**

1. **Calibrate motors:**
See [Motor Calibration Guide](../calibration/motor_calibration.md)

2. **Check mechanical issues:**
- Wheel friction or binding
- Loose motor mount
- Encoder misalignment
- Different wheel diameters

3. **Test motor individually:**
```bash
> o 150 0 0 0  # Test FL
> o 0 150 0 0  # Test FR
> o 0 0 150 0  # Test RL
> o 0 0 0 150  # Test RR
```

---

## Communication Issues

### Serial Timeout

**Symptoms:**
- "Failed to reset encoders"
- Controller fails to start
- No response from ESP32

**Solutions:**

1. **Check serial connection:**
```bash
screen /dev/ttyUSB0 115200
# Type commands manually
> b
# Should respond with: 115200
```

2. **Check baud rate:**
```bash
# ESP32 should be 115200
# Verify in config.h:
grep BAUDRATE ~/dummybot-ros2-encoder/firmware/esp32/config.h
```

3. **Close conflicting connections:**
```bash
# Check if screen is already open
ps aux | grep screen
sudo killall screen

# Check if other process using port
lsof | grep ttyUSB
```

4. **Reset ESP32:**
- Press EN button on ESP32
- Or power cycle the board

### ROS2 Can't Connect to Serial

**Symptoms:**
- Hardware interface fails to configure
- "Serial port not available" error

**Solutions:**

1. **Check port in URDF:**
```bash
grep serial_port ~/dummybot-ros2-encoder/ros2_ws/src/dummybot_bringup/description/urdf/dummybot.ros2_control.xacro
# Should be: /dev/ttyUSB0
```

2. **Verify device exists:**
```bash
ls -l /dev/ttyUSB*
# Should show: crw-rw---- 1 root dialout
```

3. **Check permissions:**
```bash
groups
# Must include 'dialout'
```

### Bridge Not Working

**Symptoms:**
- Teleop doesn't work
- No response to cmd_vel

**Solutions:**

1. **Check bridge is running:**
```bash
ps aux | grep cmd_vel_bridge
# Should show python3 process
```

2. **Restart bridge:**
```bash
pkill -f cmd_vel_bridge
python3 ~/dummybot-ros2-encoder/scripts/utils/cmd_vel_bridge.py
```

3. **Verify topics:**
```bash
ros2 topic list | grep cmd_vel
# Should show:
# /cmd_vel
# /diff_drive_controller/cmd_vel
```

4. **Test bridge manually:**
```bash
# Publish to /cmd_vel
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}}"
  
# Check if converted to stamped
ros2 topic echo /diff_drive_controller/cmd_vel
```

---

## Motor Control Issues

### Robot Doesn't Respond to Teleop

**Symptoms:**
- Teleop runs but robot doesn't move
- No errors shown

**Solutions:**

1. **Check controller status:**
```bash
ros2 control list_controllers
# Both should be 'active':
# - joint_state_broadcaster
# - diff_drive_controller
```

2. **Check topic connection:**
```bash
ros2 topic info /diff_drive_controller/cmd_vel
# Should show:
# Subscription count: 1 (controller listening)
# Publisher count: 1 (bridge or teleop)
```

3. **Test with direct publish:**
```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.2}}}" --rate 10
```

4. **Check velocity limits:**
```bash
# Verify config allows motion
grep "max_velocity" ~/dummybot-ros2-encoder/ros2_ws/src/dummybot_bringup/bringup/config/dummybot_controllers.yaml
```

### Rotation Too Slow

**Symptoms:**
- Linear motion OK
- Rotation barely happens
- High angular velocity needed

**Solutions:**

1. **Increase angular velocity limit:**
```bash
nano ~/dummybot-ros2-encoder/ros2_ws/src/dummybot_bringup/bringup/config/dummybot_controllers.yaml
# Change:
# angular.z.max_velocity: 20.0  # Increase if needed
```

2. **Use higher turn speed in teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p turn:=15.0
```

3. **Check conversion math:**
The issue may be in `radians_per_sec_to_ticks_per_frame` conversion in hardware interface.

### PID Oscillation

**Symptoms:**
- Motors oscillate around target speed
- Jerky motion
- Whining sound from motors

**Solutions:**

1. **Reduce Kp gain:**
```bash
screen /dev/ttyUSB0 115200
> p  # Check current values
> u 40 20 0 15  # Lower Kp from 60 to 40
```

2. **Increase Kd damping:**
```bash
> u 60 30 0 15  # Increase Kd from 20 to 30
```

3. **Test incrementally:**
```bash
# Start with low gains
> u 20 10 0 10
# Gradually increase until stable
```

### Auto-Stop Activating

**Symptoms:**
- Robot stops after 2 seconds
- "AUTO-STOP: No command received" in serial

**Solutions:**

1. **Check command frequency:**
```bash
ros2 topic hz /diff_drive_controller/cmd_vel
# Should be > 0.5 Hz (every 2 seconds)
```

2. **Increase timeout (in ESP32):**
```cpp
// In config.h, change:
#define AUTO_STOP_INTERVAL 5000  // 5 seconds instead of 2
```

3. **Disable auto-stop temporarily:**
```bash
# Comment out auto-stop check in main loop
# Re-upload firmware
```

---

## ROS2 Issues

### Build Failures

**Symptoms:**
- `colcon build` fails
- Missing dependencies

**Solutions:**

1. **Clean and rebuild:**
```bash
cd ~/dummybot-ros2-encoder/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

2. **Install dependencies:**
```bash
cd ~/dummybot-ros2-encoder/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Check ROS2 sourcing:**
```bash
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO
# Should show: jazzy
```

### Controller Manager Fails

**Symptoms:**
- "Failed to configure hardware"
- Segmentation fault
- Controller won't load

**Solutions:**

1. **Check hardware interface compilation:**
```bash
cd ~/dummybot-ros2-encoder/ros2_ws
colcon build --packages-select dummybot_bringup --cmake-clean-cache
```

2. **Verify URDF is valid:**
```bash
ros2 run xacro xacro ~/dummybot-ros2-encoder/ros2_ws/src/dummybot_bringup/description/urdf/dummybot.urdf.xacro
# Should output valid URDF without errors
```

3. **Check controller config:**
```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('ros2_ws/src/dummybot_bringup/bringup/config/dummybot_controllers.yaml'))"
```

### Odometry Drift

**Symptoms:**
- Robot position in RViz doesn't match reality
- Increasing error over time
- Wrong map location

**Solutions:**

1. **Calibrate motors:**
See [Motor Calibration Guide](../calibration/motor_calibration.md)

2. **Check wheel parameters:**
```bash
# Verify these match physical robot:
grep -E "wheel_radius|wheel_separation" \
  ~/dummybot-ros2-encoder/ros2_ws/src/dummybot_bringup/bringup/config/dummybot_controllers.yaml
```

3. **Check encoder resolution:**
```bash
# Should be 1760 ticks/rev
grep TICKS_PER_REV ~/dummybot-ros2-encoder/firmware/esp32/config.h
```

4. **Test encoder accuracy:**
```bash
# Rotate wheel exactly 10 times, check encoder count
# Should be: 10 × 1760 = 17600 ± 20 ticks
```

---

## Navigation Issues

### LiDAR Not Publishing

**Symptoms:**
- No `/scan` topic
- RViz shows no laser data

**Solutions:**

1. **Check LiDAR connection:**
```bash
ls /dev/ttyUSB*
# LiDAR typically on /dev/ttyUSB1 if ESP32 on ttyUSB0
```

2. **Launch LiDAR node:**
```bash
ros2 launch ldlidar_node ld06.launch.py
```

3. **Check topic:**
```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

### SLAM Not Working

**Symptoms:**
- Map not building
- Robot position jumps
- No map in RViz

**Solutions:**

1. **Check odometry:**
```bash
ros2 topic hz /diff_drive_controller/odom
# Should be ~30 Hz
```

2. **Verify TF tree:**
```bash
ros2 run tf2_tools view_frames
# Check odom → base_link → laser frame
```

3. **Check SLAM parameters:**
```bash
# Verify SLAM config in dummybot_navigation
```

---

## Getting Help

### Collect Debug Information
```bash
# ROS2 version
ros2 --version

# List nodes
ros2 node list

# List topics
ros2 topic list

# Controller status
ros2 control list_controllers

# ESP32 status
screen /dev/ttyUSB0 115200
> p  # PID params
> e  # Encoder values
```

### Log Files
```bash
# ROS2 logs
~/.ros/log/latest/

# Check specific node
ros2 run rqt_console rqt_console
```

### Report Issues

When reporting issues on GitHub, include:
1. Hardware setup (motors, drivers, ESP32 version)
2. Software versions (ROS2, Ubuntu, firmware date)
3. Error messages (full output)
4. Steps to reproduce
5. Debug information from above

---

## Quick Reference

### Restart Everything
```bash
# Kill all ROS nodes
pkill -9 -f ros

# Reset ESP32
# Press EN button or power cycle

# Restart robot
cd ~/dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 launch dummybot_bringup dummybot.launch.py

# Start bridge
python3 scripts/utils/cmd_vel_bridge.py

# Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Emergency Stop
```bash
# Serial
screen /dev/ttyUSB0 115200
> s

# Or kill ROS
pkill -9 -f ros
```
