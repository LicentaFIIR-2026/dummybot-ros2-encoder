# DummyBot Hardware Specifications

Complete hardware specifications and wiring diagrams for DummyBot.

## Robot Dimensions

| Parameter | Value | Notes |
|-----------|-------|-------|
| Wheel Diameter | 65 mm | 0.065 m |
| Wheel Radius | 32.5 mm | 0.0325 m |
| Wheel Track (Width) | 386.3 mm | 0.3863 m (center-to-center) |
| Wheelbase (Length) | 372.32 mm | 0.37232 m (center-to-center) |
| Wheel Circumference | 204.2 mm | 2π × radius |
| Drive Type | 4WD | Independent control |

## Motor System

### Motors
- **Type**: DC Geared Motors
- **Gear Ratio**: 40:1
- **Quantity**: 4 (FL, FR, RL, RR)
- **Voltage**: 6-12V DC
- **Control**: PWM (0-255)

### Motor Drivers
- **Type**: L298N H-Bridge
- **Quantity**: 2 (one per side)
- **Max Current**: 2A per channel
- **Logic Voltage**: 5V
- **Motor Voltage**: 7-12V
- **PWM Frequency**: 20 kHz

### Power Distribution
```
Battery (7-12V)
    ├─> L298N Driver 1 (Left: FL + RL)
    └─> L298N Driver 2 (Right: FR + RR)

5V Power (from Buck Converter)
    ├─> ESP32
    ├─> Encoders (×4)
    └─> Raspberry Pi 5 (via separate 5V/5A supply)
```

## Encoder System

### Specifications
- **Type**: Quadrature Incremental Encoders
- **PPR**: 11 pulses per motor shaft revolution
- **CPR**: 44 counts per revolution (quadrature)
- **Gear Ratio**: 40:1
- **Ticks per Wheel Revolution**: 1760 (44 × 40)
- **Resolution**: 0.116 mm per tick
- **Voltage**: 3.3-5V

### Encoder Channels
Each encoder has two channels (A and B) for direction detection.

## Microcontroller: ESP32

### Board
- **Model**: ESP32 DevKit v1
- **CPU**: Dual-core Xtensa LX6 @ 240 MHz
- **RAM**: 520 KB
- **Flash**: 4 MB
- **GPIO**: 34 pins
- **ADC**: 18 channels, 12-bit
- **PWM**: 16 channels
- **UART**: 3 ports
- **I2C**: 2 ports
- **Voltage**: 3.3V logic, 5V USB power

### Pin Usage Summary

| Function | Pins Used | Total |
|----------|-----------|-------|
| Motor PWM | 6 | ENA_L, ENB_L, ENA_R, ENB_R |
| Motor Direction | 8 | IN1-4 (Left), IN1-4 (Right) |
| Encoders | 8 | 4×A + 4×B channels |
| **Total** | **22** | |

## Wiring Diagrams

### L298N Driver 1 (Left Side)
```
L298N #1 (Left)          ESP32
┌─────────────┐       ┌──────────┐
│             │       │          │
│ ENA ────────┼───────┤ GPIO 13  │ (PWM)
│ IN1 ────────┼───────┤ GPIO 12  │
│ IN2 ────────┼───────┤ GPIO 14  │
│ IN3 ────────┼───────┤ GPIO 27  │
│ IN4 ────────┼───────┤ GPIO 26  │
│ ENB ────────┼───────┤ GPIO 25  │ (PWM)
│             │       │          │
│ OUT1 ───> FL Motor  │          │
│ OUT2 ───> FL Motor  │          │
│ OUT3 ───> RL Motor  │          │
│ OUT4 ───> RL Motor  │          │
│             │       │          │
│ 12V ────────┼───< Battery +    │
│ GND ────────┼───< Battery -    │
│ 5V ─────────┼───> ESP32 VIN    │
└─────────────┘       └──────────┘
```

### L298N Driver 2 (Right Side)
```
L298N #2 (Right)         ESP32
┌─────────────┐       ┌──────────┐
│             │       │          │
│ ENA ────────┼───────┤ GPIO 15  │ (PWM)
│ IN1 ────────┼───────┤ GPIO 2   │
│ IN2 ────────┼───────┤ GPIO 4   │
│ IN3 ────────┼───────┤ GPIO 18  │
│ IN4 ────────┼───────┤ GPIO 5   │
│ ENB ────────┼───────┤ GPIO 19  │ (PWM)
│             │       │          │
│ OUT1 ───> FR Motor  │          │
│ OUT2 ───> FR Motor  │          │
│ OUT3 ───> RR Motor  │          │
│ OUT4 ───> RR Motor  │          │
└─────────────┘       └──────────┘
```

### Encoder Connections
```
Encoder         ESP32           Notes
─────────────────────────────────────────
FL_A        →   GPIO 32         Interrupt
FL_B        →   GPIO 35         Input only
FR_A        →   GPIO 21         Interrupt
FR_B        →   GPIO 33         Interrupt
RL_A        →   GPIO 16         Interrupt
RL_B        →   GPIO 17         Interrupt
RR_A        →   GPIO 22         Interrupt
RR_B        →   GPIO 23         Interrupt

All encoders:
VCC         →   5V (from ESP32 VIN or external)
GND         →   GND
```

## Sensors

### LiDAR: LD06
- **Type**: 360° 2D LiDAR
- **Range**: 0.15m - 12m
- **Angular Resolution**: 1°
- **Scan Rate**: 4500 RPM
- **Interface**: UART (230400 baud)
- **Power**: 5V, 400mA
- **Package**: `ldrobot_lidar_ros2`

### Camera (Optional)
- **Type**: USB Webcam or Pi Camera
- **Resolution**: 640×480 @ 30fps
- **Use**: MediaPipe gesture control
- **Package**: `media_pipe_ros2`

## Compute Platform

### Raspberry Pi 5
- **RAM**: 16GB
- **CPU**: Quad-core Cortex-A76 @ 2.4 GHz
- **Storage**: 64GB+ MicroSD
- **OS**: Ubuntu 24.04 Noble (arm64)
- **ROS**: ROS2 Jazzy Jalisco
- **Connectivity**: 
  - WiFi 5 (802.11ac)
  - Bluetooth 5.0
  - Gigabit Ethernet
  - 2× USB 3.0, 2× USB 2.0

## Communication Architecture
```
┌─────────────────┐
│  Raspberry Pi 5 │
│   (ROS2 Jazzy)  │
└────────┬────────┘
         │ USB Serial
         │ (115200 baud)
         ↓
    ┌────────┐
    │ ESP32  │
    │ Bridge │
    └───┬────┘
        │
    ┌───┴────────────┬──────────┬──────────┐
    ↓                ↓          ↓          ↓
┌────────┐    ┌─────────┐  ┌────────┐  ┌────────┐
│L298N #1│    │L298N #2 │  │Encoder │  │Encoder │
│(Left)  │    │(Right)  │  │ FL+RL  │  │ FR+RR  │
└────────┘    └─────────┘  └────────┘  └────────┘
    │              │            │           │
    ↓              ↓            ↓           ↓
FL + RL      FR + RR       Feedback    Feedback
 Motors       Motors
```

## Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Max Linear Speed | ~1.0 m/s | Configurable in ROS |
| Max Angular Speed | ~20.0 rad/s | For rotation |
| Control Frequency | 30 Hz | PID loop rate |
| Odometry Update | 30 Hz | Encoder reading rate |
| Position Accuracy | ±0.116 mm | Per encoder tick |
| Battery Life | 2-3 hours | Depends on usage |

## Bill of Materials (BOM)

### Main Components
| Item | Quantity | Notes |
|------|----------|-------|
| Raspberry Pi 5 16GB | 1 | Main compute |
| ESP32 DevKit | 1 | Motor controller |
| L298N Motor Driver | 2 | H-bridge modules |
| DC Geared Motor 6-12V | 4 | With encoders |
| LD06 LiDAR | 1 | 360° scanning |
| 4WD Robot Chassis | 1 | Frame and wheels |
| 12V Battery | 1 | LiPo or NiMH |
| Buck Converter 12V→5V | 1 | Power regulation |
| Jumper Wires | Set | Various lengths |
| USB Cable | 1 | ESP32 connection |

## Safety Considerations

1. **Power Supply**
   - Use proper fuses
   - Separate logic and motor power
   - Add capacitors near motor drivers

2. **Motor Drivers**
   - L298N can get hot, consider heatsinks
   - Don't exceed 2A per channel
   - Use flyback diodes (included in L298N)

3. **Encoders**
   - Use 5V for encoders, not 3.3V
   - Add pull-up resistors if needed
   - Keep encoder wires short and twisted

4. **Software**
   - Auto-stop enabled (2s timeout)
   - Emergency stop command (`s`)
   - Velocity limits in ROS config

## Mechanical Assembly Notes

1. Mount L298N drivers securely with good ventilation
2. Keep encoder wires away from motor power wires
3. Use cable management for clean wiring
4. Ensure wheels are properly aligned
5. Center of mass should be low and centered
6. Allow space for battery and electronics

## Next Steps

- [Motor Calibration](../calibration/motor_calibration.md)
- [Troubleshooting Guide](../troubleshooting/common_issues.md)
- [Installation Guide](../setup/installation.md)
