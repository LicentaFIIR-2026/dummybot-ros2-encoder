# USB Device Mapping - udev Rules

## Overview
This setup ensures ESP32 and LD06 LiDAR always map to the same device names, regardless of connection order.

## Device Mapping
- **ESP32**: `/dev/ttyESP32` (physical port: usb2/2-1)
- **LD06 LiDAR**: `/dev/ttyLIDAR` (physical port: usb4/4-2)

## Installation

### 1. Create udev rules file
```bash
sudo nano /etc/udev/rules.d/99-dummybot-serial.rules
```

### 2. Add rules
```bash
# ESP32 - Port USB usb2/2-1 -> /dev/ttyESP32
SUBSYSTEM=="tty", KERNELS=="2-1:1.0", SYMLINK+="ttyESP32", MODE="0666"

# LiDAR LD06 - Port USB usb4/4-2 -> /dev/ttyLIDAR
SUBSYSTEM=="tty", KERNELS=="4-2:1.0", SYMLINK+="ttyLIDAR", MODE="0666"
```

### 3. Activate rules
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 4. Verify
```bash
ls -l /dev/ttyESP32 /dev/ttyLIDAR
```

## Troubleshooting

If ports change after reboot, identify new physical ports:
```bash
udevadm info --name=/dev/ttyUSB0 | grep "DEVPATH"
udevadm info --name=/dev/ttyUSB1 | grep "DEVPATH"
```

Update KERNELS value in udev rules accordingly (e.g., `2-1:1.0` → `2-2:1.0`).
