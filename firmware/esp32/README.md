# DummyBot ESP32 Firmware

ESP32 bridge for ROS2 communication with 4-wheel independent motor control and encoder reading.

## Hardware Specifications

### Motor Configuration
- **Type**: 4x DC motors with 40:1 gearbox
- **Drivers**: 2x L298N motor drivers
- **Control**: Independent PWM control for each wheel
- **Encoders**: 4x Quadrature encoders (11 PPR, 1760 ticks/rev after gearing)

### Pin Connections

#### L298N Driver 1 (Left Side: FL + RL)
| Pin | GPIO | Function |
|-----|------|----------|
| ENA_L | 13 | PWM for left motors |
| IN1_L | 12 | FL direction |
| IN2_L | 14 | FL direction |
| IN3_L | 27 | RL direction |
| IN4_L | 26 | RL direction |
| ENB_L | 25 | PWM for left motors |

#### L298N Driver 2 (Right Side: FR + RR)
| Pin | GPIO | Function |
|-----|------|----------|
| ENA_R | 15 | PWM for FR motor |
| IN1_R | 2 | FR direction |
| IN2_R | 4 | FR direction |
| IN3_R | 18 | RR direction |
| IN4_R | 5 | RR direction |
| ENB_R | 19 | PWM for RR motor |

#### Encoders
| Motor | A Pin | B Pin |
|-------|-------|-------|
| Front Left (FL) | GPIO 32 | GPIO 35 |
| Front Right (FR) | GPIO 21 | GPIO 33 |
| Rear Left (RL) | GPIO 16 | GPIO 17 |
| Rear Right (RR) | GPIO 22 | GPIO 23 |

## Serial Protocol

**Baudrate**: 115200  
**Format**: ASCII commands with newline terminator

### Available Commands

| Command | Format | Description | Response Example |
|---------|--------|-------------|------------------|
| `b` | `b` | Get baudrate | `115200` |
| `e` | `e` | Read all encoders | `1234 -567 890 -123` |
| `r` | `r` | Reset encoders to zero | `OK` |
| `m` | `m FL FR RL RR` | Set motor speeds with PID (ticks/frame) | `OK` |
| `o` | `o FL FR RL RR` | Set raw PWM (-255 to 255) | `OK` |
| `u` | `u Kp Kd Ki Ko` | Update PID parameters | `OK` |
| `p` | `p` | Get current PID parameters | `Kp:60 Kd:20 Ki:0 Ko:15` |
| `c` | `c FL FR RL RR` | Set calibration factors | `OK` |
| `C` | `C` | Reset calibration to defaults | `OK` |
| `s` | `s` | Auto-stop (emergency stop) | `OK` |
| `t` | `t` | Test motors individually | `OK` |

### Command Examples
```bash
# Read encoders
> e
12345 -6789 23456 -34567

# Move forward (50 ticks/frame all motors)
> m 50 50 50 50
OK

# Rotate in place
> m 100 -100 100 -100
OK

# Raw PWM control (150 PWM forward)
> o 150 150 150 150
OK

# Update PID gains
> u 60 20 0 15
OK

# Set calibration (FR needs 1.67x more power)
> c 1.0 1.67 1.0 1.0
OK
```

## PID Configuration

### Default Parameters
- **Kp**: 60 (Proportional gain)
- **Kd**: 20 (Derivative gain)
- **Ki**: 0 (Integral gain)
- **Ko**: 15 (Output scaling)
- **Loop Rate**: 30 Hz (every 33ms)

### PWM Configuration
- **Frequency**: 20 kHz
- **Resolution**: 8-bit (0-255)
- **Max PWM**: ±255

## Encoder Specifications

- **PPR (Pulses Per Revolution)**: 11 per motor shaft
- **Gear Ratio**: 40:1
- **CPR (Counts Per Revolution)**: 44 (quadrature: 11×4)
- **Ticks Per Wheel Revolution**: 1760 (44×40)
- **Distance Per Tick**: 0.116 mm

## Robot Dimensions

- **Wheel Diameter**: 65 mm (0.065 m)
- **Wheel Radius**: 32.5 mm (0.0325 m)
- **Wheel Track (L)**: 386.3 mm (0.3863 m)
- **Wheelbase (W)**: 372.32 mm (0.37232 m)

## Motor Calibration

Calibration factors compensate for mechanical differences between motors. Values are stored in EEPROM and persist across reboots.

**Current calibration:**
- FL (Front Left): 1.0
- FR (Front Right): 1.67 *(needs more power)*
- RL (Rear Left): 1.0
- RR (Rear Right): 1.0

To recalibrate, use the `c` command with new factors.

## Uploading Firmware

### Using Arduino IDE

1. Install ESP32 board support:
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
2. Tools → Board → ESP32 Arduino → ESP32 Dev Module
3. Tools → Upload Speed → 115200
4. Select correct COM port
5. Upload `DummyBot_ESP32_Bridge.ino`

### Using PlatformIO
```bash
cd firmware/esp32
pio run -t upload
pio device monitor -b 115200
```

## Safety Features

- **Auto-Stop**: Motors stop automatically if no command received for 2 seconds
- **PWM Limiting**: Output automatically clamped to ±255
- **Integral Windup Protection**: Prevents I-term accumulation at saturation

## Troubleshooting

### Motors don't move
- ✓ Check power supply (7-12V, sufficient current)
- ✓ Verify L298N enable jumpers are in place
- ✓ Test with raw PWM: `o 150 150 150 150`
- ✓ Check motor connections and polarity

### Encoders not reading
- ✓ Verify encoder power (5V)
- ✓ Check pull-up resistors on encoder outputs
- ✓ Test individual channels with `e` command
- ✓ Ensure encoder wiring is correct (A/B channels)

### One motor slower/faster
- ✓ Use calibration command `c` to compensate
- ✓ Check for mechanical binding or friction
- ✓ Verify motor driver connections

### Serial communication issues
- ✓ Verify baudrate is 115200
- ✓ Check USB cable quality
- ✓ Try different USB port
- ✓ Use `screen /dev/ttyUSB0 115200` to test

## Debug Mode

Uncomment in `config.h` for additional debugging:
```cpp
#define DEBUG_ENCODERS  // Print encoder values
#define DEBUG_PID       // Print PID calculations
#define DEBUG_MOTORS    // Print motor commands
```

## Files Structure

- `DummyBot_ESP32_Bridge.ino` - Main program loop
- `config.h` - Hardware configuration and parameters
- `commands.h` - Serial protocol definitions
- `motor_driver_l298n.{h,ino}` - Motor control driver
- `encoder_driver.{h,ino}` - Encoder reading with interrupts
- `diff_controller.{h,ino}` - PID control implementation
