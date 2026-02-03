# MCP Navigation System v1.0

🤖 AI-powered waypoint navigation for DummyBot using Claude Desktop integration.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Terminal Setup](#terminal-setup)
- [Quick Start Guide](#quick-start-guide)
- [Voice Commands](#voice-commands)
- [System Files](#system-files)
- [Adding New Waypoints](#adding-new-waypoints)
- [ROS2 Services](#ros2-services)
- [Dependencies](#dependencies)
- [Troubleshooting](#troubleshooting)
- [Version History](#version-history)

---

## 🎯 Overview

This system allows **voice and text control** of the DummyBot autonomous mobile robot through **Claude Desktop**, using ROS2 services and the ros-mcp-server bridge. Simply say *"Du robotul la home"* and the robot navigates autonomously!

### Key Features
- ✅ Natural language waypoint navigation via Claude AI
- ✅ Interactive waypoint recording with live AMCL feedback
- ✅ Persistent waypoint storage in YAML format
- ✅ Automatic coordinate conversion (yaw degrees → quaternion)
- ✅ ROS2 service-based architecture for reliability
- ✅ Integration with Nav2 navigation stack

---

## 🏗️ Architecture
```
┌─────────────────────────────────────────────────────────────┐
│                    USER (Voice/Text)                         │
│                "Du robotul la usa-spate"                     │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              Claude Desktop (PC Local)                       │
│           Natural Language Processing                        │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              ros-mcp-server (Bridge)                         │
│       Translates AI commands → ROS2 service calls           │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼ ROS2 Service Call
┌─────────────────────────────────────────────────────────────┐
│         waypoint_service_node.py (Robot)                    │
│  • Reads named_waypoints.yaml                               │
│  • Converts "usa-spate" → (1.103, 2.728, -102.4°)          │
│  • Converts yaw → quaternion                                │
│  • Publishes to /goal_pose                                  │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼ PoseStamped on /goal_pose
┌─────────────────────────────────────────────────────────────┐
│         Nav2 Navigation Stack (navxplorer)                  │
│  • AMCL localization                                        │
│  • Path planning (avoiding obstacles)                       │
│  • Controller (generates /cmd_vel)                          │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼ /cmd_vel commands
┌─────────────────────────────────────────────────────────────┐
│         Robot Hardware (robot.launch.py)                    │
│  • Differential drive controller                            │
│  • Motor drivers (ESP32)                                    │
│  • Wheel encoders                                           │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼ Physical motion
┌─────────────────────────────────────────────────────────────┐
│                  DummyBot Robot 🤖                          │
│              Moves to destination!                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 💻 Terminal Setup

### ⚠️ CRITICAL: You need **4 terminals** running simultaneously

#### **Terminal 1: Robot Hardware & Drivers**
```bash
# Navigate to workspace
cd ~/dummybot-ros2-encoder/ros2_ws

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch robot hardware
ros2 launch dummybot_bringup robot.launch.py
```

**What it does:**
- Starts motor controllers (ESP32 interface)
- Initializes sensors (LD19 LiDAR, HD Pro Webcam C920)
- Publishes `/odom` (wheel odometry)
- Publishes `/scan` (LiDAR data)
- Accepts `/cmd_vel` commands for motion
- Manages `ros2_control` hardware interface

**Must stay running!** Do not close this terminal.

---

#### **Terminal 2: Nav2 Navigation Stack**
```bash
# Navigate to workspace
cd ~/dummybot-ros2-encoder/ros2_ws

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch navigation with AMCL localization
ros2 launch amr2ax_nav2 navxplorer.launch.py \
  localization_type:=2D \
  slam:=False \
  use_sim_time:=False
```

**What it does:**
- **AMCL**: Adaptive Monte Carlo Localization (tracks robot position on map)
- **Planner Server**: Calculates collision-free paths
- **Controller Server**: Converts paths to `/cmd_vel` commands
- **Costmaps**: Local and global obstacle detection
- **Behavior Trees**: High-level navigation logic
- **Recovery Behaviors**: Handles stuck situations

**Must stay running!** Do not close this terminal.

---

#### **Terminal 3: Waypoint Service Node** ⭐ (NEW!)
```bash
# Navigate to workspace
cd ~/dummybot-ros2-encoder/ros2_ws

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run waypoint service node
python3 ~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/scripts/waypoint_service_node.py
```

**What it does:**
- Reads `named_waypoints.yaml` (waypoint database)
- Creates ROS2 services:
  - `/navigate_to_home`
  - `/navigate_to_usa_spate`
  - `/navigate_to_hol_spate`
  - `/list_waypoints`
- Converts waypoint names → coordinates → `/goal_pose` messages
- Handles yaw (degrees) → quaternion conversion

**Expected output:**
```
[INFO] [waypoint_service_node]: Serviciu creat: /navigate_to_usa_spate
[INFO] [waypoint_service_node]: Serviciu creat: /navigate_to_hol_spate
[INFO] [waypoint_service_node]: Serviciu creat: /navigate_to_home
[INFO] [waypoint_service_node]: Waypoint Service Node pornit cu 3 waypoint-uri
```

**Must stay running!** Do not close this terminal.

---

#### **Terminal 4: Claude Desktop** (or monitoring)

**Option A: Open Claude Desktop application**
- ros-mcp-server starts automatically (configured in `~/.config/Claude/claude_desktop_config.json`)
- No terminal needed, just use the GUI

**Option B: Use this terminal for monitoring/debugging**
```bash
# Monitor navigation status
ros2 topic echo /goal_pose

# Check active services
ros2 service list | grep navigate

# Monitor robot position
ros2 topic echo /amcl_pose

# Check if waypoint node is running
ros2 node list | grep waypoint
```

---

## 🚀 Quick Start Guide

### Step 1: Power on the robot
- Connect power supply to DummyBot
- Verify ESP32 boards are powered
- Ensure LiDAR is spinning

### Step 2: Launch all terminals (in order)

1. **Terminal 1:** `ros2 launch dummybot_bringup robot.launch.py`
2. **Terminal 2:** `ros2 launch amr2ax_nav2 navxplorer.launch.py ...` (wait for AMCL to initialize)
3. **Terminal 3:** `python3 waypoint_service_node.py` (wait for services to appear)
4. **Open Claude Desktop** (ros-mcp-server auto-starts)

### Step 3: Initialize robot localization

**Option A: Use RViz**
```bash
rviz2
```
- Click "2D Pose Estimate"
- Click on map where robot is located
- Drag arrow in direction robot is facing

**Option B: Publish initial pose manually**
```bash
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "..."
```

### Step 4: Verify system is ready
```bash
# Check all nodes are running
ros2 node list

# Should see:
# /waypoint_service_node
# /amcl
# /planner_server
# /controller_server
# (and many more)

# Check services exist
ros2 service list | grep navigate

# Should see:
# /navigate_to_home
# /navigate_to_hol_spate
# /navigate_to_usa_spate
# /list_waypoints
```

### Step 5: Test with Claude Desktop!

Open Claude Desktop and say:

> *"Listează serviciile ROS2 care conțin 'navigate'"*

Claude should respond with the list of services.

Then:

> *"Du robotul la home"*

Robot should start moving! 🎉

---

## 🎤 Voice Commands

### Navigation Commands

| Command | Description | What happens |
|---------|-------------|--------------|
| *"Du robotul la home"* | Navigate to home position | Calls `/navigate_to_home` service |
| *"Navighează la usa-spate"* | Go to back door | Calls `/navigate_to_usa_spate` service |
| *"Mergi la hol-spate"* | Go to back hallway | Calls `/navigate_to_hol_spate` service |
| *"Trimite robotul acasă"* | Return home | Calls `/navigate_to_home` service |

### Query Commands

| Command | Description |
|---------|-------------|
| *"Listează waypoint-urile disponibile"* | Shows all saved waypoints |
| *"Ce servicii de navigație sunt active?"* | Lists ROS2 navigation services |
| *"Unde poate merge robotul?"* | Shows available destinations |

### Technical Commands (for debugging)

| Command | Description |
|---------|-------------|
| *"Apelează serviciul /navigate_to_home"* | Directly call service |
| *"Verifică dacă nodul waypoint_service_node rulează"* | Check node status |
| *"Arată-mi topicurile ROS2 active"* | List all topics |

---

## 📁 System Files

### Core Scripts

| File | Purpose | Location |
|------|---------|----------|
| `waypoint_service_node.py` | Main ROS2 service node for Claude integration | `src/amr2ax_nav2/scripts/` |
| `waypoint_nav.py` | Interactive waypoint recording/navigation tool | `src/amr2ax_nav2/scripts/` |
| `get_waypoint_coords.py` | Coordinate extraction helper | `src/amr2ax_nav2/scripts/` |
| `export_waypoints_json.py` | JSON export utility | `src/amr2ax_nav2/scripts/` |
| `waypoint_mcp.py` | Direct MCP server (alternative approach) | `src/amr2ax_nav2/scripts/` |

### Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| `named_waypoints.yaml` | Persistent waypoint storage | `src/amr2ax_nav2/config/` |
| `xplorer.yaml` | Nav2 parameters | `src/amr2ax_nav2/config/` |
| `README_WAYPOINTS.md` | Claude Desktop integration docs | `src/amr2ax_nav2/scripts/` |

### Current Waypoints (v1.0)
```yaml
waypoints:
  usa-spate:
    x: 1.103
    y: 2.728
    yaw: -102.4
  hol-spate:
    x: 4.293
    y: 2.709
    yaw: 0.7
  home:
    x: -0.754
    y: -1.425
    yaw: -178.0
```

**Coordinate System:**
- `x`, `y`: Position in meters (frame_id: `map`)
- `yaw`: Orientation in degrees (0° = facing +X axis)
- All coordinates relative to map origin

---

## 📝 Adding New Waypoints

### Method 1: Interactive Recording (Recommended)
```bash
# Start recording mode
python3 ~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/scripts/waypoint_nav.py record
```

**Steps:**
1. Script shows live robot position: `📍 Poziție: X=4.172m  Y=2.835m  Yaw=45.3°`
2. Move robot to desired location (using teleop or manually)
3. Press **Enter** when positioned correctly
4. Enter waypoint name (e.g., `bucatarie`, `dormitor`)
5. Waypoint is saved to `named_waypoints.yaml`
6. Repeat for more locations
7. Press **Ctrl+C** to exit

### Method 2: Manual YAML Edit
```bash
nano ~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml
```

Add new waypoint:
```yaml
waypoints:
  bucatarie:
    x: 3.456
    y: 1.234
    yaw: 90.0
```

**Important:** After manual edit, restart `waypoint_service_node.py` to load changes!

### Method 3: Get Current Position
```bash
# Get robot's current position
ros2 topic echo /amcl_pose --once
```

Extract `x`, `y` from `position` and convert `orientation` quaternion to yaw degrees.

---

## 🔧 ROS2 Services Created

| Service Name | Type | Description |
|--------------|------|-------------|
| `/navigate_to_home` | `std_srvs/srv/Trigger` | Navigate to home waypoint |
| `/navigate_to_usa_spate` | `std_srvs/srv/Trigger` | Navigate to back door |
| `/navigate_to_hol_spate` | `std_srvs/srv/Trigger` | Navigate to back hallway |
| `/list_waypoints` | `std_srvs/srv/Trigger` | List all waypoints with coordinates |

### Manual Service Testing
```bash
# Test navigation to home
ros2 service call /navigate_to_home std_srvs/srv/Trigger

# Expected response:
# success: True
# message: 'Navigare la home: (-0.75, -1.43, -178.0°)'

# List waypoints
ros2 service call /list_waypoints std_srvs/srv/Trigger
```

---

## 📦 Dependencies

### ROS2 Packages
- **ROS2 Jazzy** (Ubuntu 24.04)
- **Nav2** - Navigation stack
- **AMCL** - Localization
- **ros2_control** - Hardware interface
- **tf2** - Transform library

### Python Libraries
```bash
pip install pyyaml
pip install transforms3d  # or python3-tf-transformations
```

### External Tools

#### ros-mcp-server
Standard [ros-mcp-server](https://github.com/ajshedivy/ros-mcp) (no modifications needed)

**Installation:**
```bash
# Install via uvx (recommended)
uvx ros-mcp --help
```

**Configuration:**
Edit `~/.config/Claude/claude_desktop_config.json` on your **local PC**:
```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "bash",
      "args": ["-lc", "uvx ros-mcp --transport=stdio"]
    }
  }
}
```

**Note:** ros-mcp-server runs on your **local machine** (where Claude Desktop is installed), not on the robot!

---

## 🐛 Troubleshooting

### Problem: Service not responding

**Symptoms:**
- Claude says "service didn't respond in time"
- `ros2 service call` hangs or times out

**Solutions:**
```bash
# Check if waypoint node is running
ros2 node list | grep waypoint

# If not listed, restart Terminal 3
python3 waypoint_service_node.py

# Verify services exist
ros2 service list | grep navigate
```

---

### Problem: Robot doesn't move

**Symptoms:**
- Service call succeeds but robot stays still
- No `/cmd_vel` commands published

**Solutions:**
```bash
# 1. Check if Nav2 is running
ros2 node list | grep planner

# 2. Verify robot is localized
ros2 topic echo /amcl_pose --once
# Should show valid position, not (0,0,0)

# 3. Check if goal was published
ros2 topic echo /goal_pose --once

# 4. Verify path planning works
ros2 topic echo /plan

# 5. In RViz, give "2D Pose Estimate" if robot is lost
```

---

### Problem: Claude doesn't see services

**Symptoms:**
- Claude says "no services found"
- ros-mcp-server not connecting

**Solutions:**
```bash
# 1. Restart Claude Desktop completely
pkill -9 Claude
# Then reopen Claude Desktop

# 2. Verify ros-mcp-server config
cat ~/.config/Claude/claude_desktop_config.json

# 3. Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # On robot
# Should be same on PC (usually 0 or unset)

# 4. Test ros-mcp-server manually
uvx ros-mcp --help
```

---

### Problem: Waypoint coordinates wrong

**Symptoms:**
- Robot navigates to wrong location
- Waypoint position doesn't match expected

**Solutions:**
```bash
# 1. Verify waypoint coordinates
cat ~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml

# 2. Check current robot position
ros2 topic echo /amcl_pose

# 3. Re-record waypoint
python3 waypoint_nav.py record
# Move robot to correct position
# Press Enter and overwrite waypoint

# 4. Restart waypoint_service_node.py to reload YAML
```

---

### Problem: AMCL not publishing position

**Symptoms:**
- `/amcl_pose` topic has no data
- `ros2 topic hz /amcl_pose` shows no rate

**Solutions:**
```bash
# 1. AMCL needs initial pose estimate
# Use RViz "2D Pose Estimate" button

# 2. Check if map is loaded
ros2 topic echo /map --once

# 3. Verify LiDAR is working
ros2 topic hz /scan

# 4. Move robot slightly (AMCL updates on motion)
```

---

## 📚 Version History

### v1.0 (2025-02-03)
**Initial Release** 🎉

**Features:**
- ✅ Voice/text control via Claude Desktop
- ✅ ROS2 service-based waypoint navigation
- ✅ Interactive waypoint recording tool
- ✅ Automatic yaw→quaternion conversion
- ✅ Integration with ros-mcp-server bridge
- ✅ 3 predefined waypoints (usa-spate, hol-spate, home)

**Components:**
- waypoint_service_node.py
- waypoint_nav.py
- get_waypoint_coords.py
- export_waypoints_json.py
- named_waypoints.yaml

**Tested on:**
- Hardware: DummyBot differential drive robot
- Sensors: LD19 LiDAR, HD Pro Webcam C920
- Software: ROS2 Jazzy, Nav2, AMCL
- Control: Claude Desktop + ros-mcp-server

---

## 🎓 Academic Information

**Project:** DummyBot Autonomous Mobile Robot  
**Topic:** AI-Enhanced Waypoint Navigation System with Natural Language Interface  
**Author:** Andrei Saim  
**Institution:** Politehnica București (UPB-FIIR)  
**Department:** Faculty of Industrial Engineering and Robotics  
**Academic Year:** 2025-2026  
**Project Type:** Bachelor's Thesis  

---

## 📄 License

[Specify your license here]

---

## 🙏 Acknowledgments

- **ros-mcp-server** by ajshedivy - MCP bridge for ROS2
- **Nav2** - ROS2 Navigation Stack
- **Anthropic Claude** - AI language model integration
- **Open Robotics** - ROS2 framework

---

## 📞 Support

For issues or questions:
1. Check the [Troubleshooting](#troubleshooting) section
2. Review terminal outputs for error messages
3. Verify all 4 terminals are running correctly
4. Contact: [Your contact info]

---

**Last Updated:** February 3, 2025  
**Version:** 1.0  
**Status:** ✅ Stable - Production Ready
