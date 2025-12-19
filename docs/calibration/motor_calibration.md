# Motor Calibration Guide

Guide for calibrating individual motors to ensure uniform robot movement.

## Why Calibration is Needed

Due to manufacturing tolerances, mechanical wear, and assembly differences, motors may not respond identically to the same PWM signal. This causes:
- **Drift during straight motion** (robot veers left or right)
- **Uneven rotation** (one side faster than the other)
- **Poor odometry accuracy**

Calibration compensates for these differences by applying motor-specific multipliers to PWM outputs.

## Current Calibration Status

Based on testing, the current calibration factors are:

| Motor | Factor | Reason |
|-------|--------|--------|
| FL (Front Left) | 1.0 | Baseline |
| FR (Front Right) | **1.67** | Slower than others, needs more power |
| RL (Rear Left) | 1.0 | Baseline |
| RR (Rear Right) | 1.0 | Baseline |

These values are stored in ESP32 EEPROM and persist across reboots.

## Calibration Process

### Prerequisites

- Robot fully assembled and powered
- ESP32 flashed with latest firmware
- Serial connection established (`screen /dev/ttyUSB0 115200`)
- Robot on blocks or free to move safely

### Step 1: Test Individual Motors

Test each motor with raw PWM to identify speed differences:
```bash
# Connect to ESP32
screen /dev/ttyUSB0 115200

# Test each motor at PWM 150 for 2 seconds (forward)
# Observe which motors are slower/faster

# Front Left
> o 150 0 0 0
[wait 2s]
> o 0 0 0 0

# Front Right
> o 0 150 0 0
[wait 2s]
> o 0 0 0 0

# Rear Left
> o 0 0 150 0
[wait 2s]
> o 0 0 0 0

# Rear Right
> o 0 0 0 150
[wait 2s]
> o 0 0 0 0
```

**Note which motors are noticeably slower or faster.**

### Step 2: Test Straight Line Motion
```bash
# All motors forward at same PWM
> o 150 150 150 150

# Observe if robot:
# - Goes straight → Good!
# - Veers left → Right motors slower, need higher factor
# - Veers right → Left motors slower, need higher factor
```

### Step 3: Test Rotation
```bash
# Rotate in place (left motors forward, right motors reverse)
> o 150 -150 150 -150

# Observe if rotation is:
# - Smooth and circular → Good!
# - Jerky or uneven → Motors need calibration
```

### Step 4: Calculate Calibration Factors

If a motor is slower, it needs a **higher** factor.

**Example calculation:**
- If FR motor needs PWM 250 to match others at PWM 150:
- Factor = 250 / 150 = **1.67**

**General formula:**
```
Calibration Factor = (PWM needed for match) / (Base PWM)
```

### Step 5: Apply Calibration
```bash
# Set calibration factors (FL, FR, RL, RR)
> c 1.0 1.67 1.0 1.0
OK

# Calibration saved to EEPROM!
```

### Step 6: Verify Calibration

Test with PID control (command `m` uses calibration):
```bash
# Reset encoders
> r
OK

# Move forward
> m 100 100 100 100
[observe straight motion]

# Rotate
> m 100 -100 100 -100
[observe smooth rotation]

# Stop
> s
OK
```

### Step 7: Fine-Tune in ROS

If calibration is close but needs refinement:
```bash
# With robot running
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Test straight line (press 'i')
# Test rotation (press 'j' or 'l')

# Adjust and re-upload to ESP32 if needed
```

## Advanced Calibration

### PID Tuning

If motors respond well but tracking is poor, adjust PID parameters:
```bash
# Check current PID
> p
Kp:60 Kd:20 Ki:0 Ko:15

# Update PID (example: increase Kp)
> u 80 20 0 15
OK
```

**PID Parameter Guidelines:**
- **Kp (Proportional)**: Affects response speed
  - Too low → Slow response, poor tracking
  - Too high → Oscillation, overshoot
  - Start: 60, adjust ±20
  
- **Kd (Derivative)**: Dampens oscillation
  - Too low → Overshoot
  - Too high → Sluggish response
  - Start: 20, adjust ±10
  
- **Ki (Integral)**: Eliminates steady-state error
  - Usually keep at 0 for mobile robots
  - Can cause windup issues
  
- **Ko (Output scaling)**: Overall gain
  - Scales final output
  - Start: 15, adjust ±5

### Per-Direction Calibration

If motors behave differently forward vs. reverse:

**Option 1: Average calibration**
```
Factor = (forward_factor + reverse_factor) / 2
```

**Option 2: Separate calibration** (requires firmware modification)
Modify `diff_controller.ino` to use different factors based on direction.

## Calibration Validation

### Test 1: Straight Line (1 meter)
```bash
# Mark start position
# Command robot forward
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.3}}}"

# After 1 meter, stop and measure deviation
# Acceptable: < 5cm deviation over 1m
```

### Test 2: Square Path
```bash
# Drive square: 1m forward, 90° turn, repeat 4×
# Robot should return to start ± 10cm
```

### Test 3: Rotation Test
```bash
# Rotate 360° in place
# Mark starting orientation
# Command full rotation
# Check if returns to same orientation ± 5°
```

## Troubleshooting

### Motor calibration doesn't save
- Check EEPROM initialization in setup()
- Verify `EEPROM.commit()` is called
- Try resetting calibration: `> C`

### Calibration makes it worse
- Reset to defaults: `> C`
- Start over with smaller adjustments (1.1, 1.2, etc.)
- Check for mechanical issues (friction, binding)

### One motor still different
- Check motor driver (may be faulty)
- Verify connections and solder joints
- Try swapping motors to isolate hardware issue
- Check encoder is working: `> e`

### Robot drifts after calibration
- May be mechanical (wheel alignment, floor surface)
- Try calibrating on different surface
- Check wheel diameter consistency
- Verify encoder mounting

## Calibration Backup

Save your calibration values for future reference:
```bash
# Read current calibration from ESP32
screen /dev/ttyUSB0 115200
> p
# Note down Kp, Kd, Ki, Ko

# Calibration factors are shown on boot
# Or check config.h:
cat ~/dummybot-ros2-encoder/firmware/esp32/config.h | grep CALIBRATION
```

**Record in robot logbook:**
```
Date: 2024-XX-XX
FL: 1.0
FR: 1.67
RL: 1.0
RR: 1.0
PID: Kp=60, Kd=20, Ki=0, Ko=15
Notes: FR motor slower due to [mechanical reason]
```

## Re-calibration Schedule

Calibrate when:
- ✓ New motor installed
- ✓ Wheels changed
- ✓ Significant drift observed
- ✓ After mechanical maintenance
- ✓ Every 3-6 months (routine)

## Next Steps

- [Troubleshooting Guide](../troubleshooting/common_issues.md)
- [Hardware Specifications](../hardware/specifications.md)
- Return to [README](../../README.md)
