# DummyBot Autonomous Navigation

Autonomous navigation system using Nav2 stack with AMCL localization and dynamic obstacle avoidance.

## System Overview

**Hardware:**
- 4WD mecanum wheels robot
- LD06 LiDAR (12m range, 360°)
- ESP32 motor control with encoders
- Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy)

**Navigation Stack:**
- **AMCL**: Adaptive Monte Carlo Localization on saved maps
- **NavFn Planner**: Global path planning (A* algorithm)
- **DWB Controller**: Dynamic Window Approach for local planning
- **Costmaps**: Static map + dynamic obstacle detection via LiDAR

## Quick Start

### 1. Launch Navigation System

**Terminal 1 - Scan Bridge (QoS compatibility):**
```bash
python3 ~/dummybot-ros2-encoder/scripts/scan_bridge.py
```
*Purpose: Converts LiDAR scan from RELIABLE to BEST_EFFORT QoS for AMCL compatibility*

**Terminal 2 - Navigation Stack:**
```bash
cd ~/dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 launch dummybot_navigation navigation.launch.py
```
*Launches: robot hardware, LiDAR, controllers, AMCL, Nav2 planning/control*

**Wait ~15 seconds for initialization**

**Terminal 3 - Cmd Vel Bridge (message type conversion):**
```bash
python3 ~/dummybot-ros2-encoder/scripts/utils/cmd_vel_bridge.py
```
*Purpose: Converts Twist → TwistStamped for diff_drive_controller*

### 2. Set Initial Pose

**Terminal 4 - Initialize AMCL:**
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
header: {frame_id: 'map'},
pose: {
  pose: {
    position: {x: -2.0, y: -1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
}
}"
```
*Adjust position (x, y) to match robot's actual starting location on map*

### 3. Verify System Status

**Check lifecycle nodes are active:**
```bash
ros2 lifecycle get /planner_server    # Should be: active [4]
ros2 lifecycle get /bt_navigator      # Should be: active [4]
ros2 lifecycle get /controller_server # Should be: active [4]
```

**If nodes are inactive, activate manually:**
```bash
ros2 lifecycle set /planner_server activate
ros2 lifecycle set /bt_navigator activate
```

### 4. Visualize & Navigate

**Terminal 5 - RViz:**
```bash
ros2 run rviz2 rviz2
```

**RViz Configuration:**
1. **Fixed Frame:** `map`
2. **Add displays:**
   - Map → Topic: `/map`, Durability: `Transient Local`
   - LaserScan → Topic: `/scan`
   - Path → Topic: `/plan` (global path - green line)
   - RobotModel
   - PoseArray → Topic: `/particle_cloud` (AMCL particles)

3. **Set goal:**
   - Toolbar: Click **"Nav2 Goal"** or **"2D Goal Pose"**
   - Click on map (white/free space)
   - Drag to set orientation
   - **Robot will navigate autonomously!**

## Architecture Details

### Bridge Requirements

**Why two bridges?**

1. **scan_bridge.py**: 
   - **Problem**: LD06 LiDAR publishes with QoS `RELIABLE`, but AMCL expects `BEST_EFFORT`
   - **Solution**: Re-publishes `/scan` → `/scan_best_effort` with correct QoS

2. **cmd_vel_bridge.py**:
   - **Problem**: Nav2 publishes `geometry_msgs/Twist`, but diff_drive_controller needs `geometry_msgs/TwistStamped`
   - **Solution**: Converts `/cmd_vel` → `/diff_drive_controller/cmd_vel` with timestamps

### Topic Flow
```
Navigation Planning:
bt_navigator → controller_server → /cmd_vel_nav → velocity_smoother → /cmd_vel

Bridging:
/cmd_vel → [cmd_vel_bridge.py] → /diff_drive_controller/cmd_vel → motors

Localization:
/scan → [scan_bridge.py] → /scan_best_effort → AMCL → /amcl_pose
```

### Key Parameters

**Robot Geometry:**
- `wheel_radius`: 0.0325 m (65mm diameter wheels)
- `wheel_separation`: 0.3863 m (38.63cm between left/right wheels)
- `robot_radius`: 0.28 m (conservative collision radius)

**Navigation Limits:**
- Max linear velocity: 0.8 m/s
- Max angular velocity: 1.5 rad/s
- Goal tolerance: 0.15 m (xy) and 0.15 rad (yaw)

**AMCL:**
- Min particles: 500
- Max particles: 2000
- Transform tolerance: 0.5 s

## Troubleshooting

### Robot doesn't move

1. **Check bridges are running**:
```bash
   ps aux | grep bridge
```

2. **Verify cmd_vel commands**:
```bash
   ros2 topic echo /diff_drive_controller/cmd_vel --once
```
   Should show velocities when navigating

3. **Check lifecycle states** (see step 3 above)

### AMCL warnings

**"Please set initial pose"**: Normal at startup, set initial pose (step 2)

**"Failed to transform"**: TF timing issue, usually resolves after initial pose set

### Navigation fails

1. **Goal in occupied space**: Choose goal in white (free) area on map
2. **Path blocked**: Robot will replan automatically
3. **"Failed to make progress"**: Goal may be unreachable, try closer goal

### Map doesn't appear in RViz

1. **Check Map display**: Durability Policy must be `Transient Local`
2. **Verify map server**:
```bash
   ros2 topic echo /map --once
```

## Files

- `config/nav2_params.yaml`: Complete Nav2 configuration
- `launch/navigation.launch.py`: Main navigation launch file
- `maps/my_map.yaml` + `my_map.pgm`: Saved map from SLAM
- `../../../scripts/scan_bridge.py`: LiDAR QoS bridge
- `../../../scripts/utils/cmd_vel_bridge.py`: Cmd_vel type converter

## Future Improvements

- [ ] Calibrate odometry for better precision
- [ ] Tune DWB controller parameters for smoother motion
- [ ] Add waypoint following for multi-goal navigation
- [ ] Implement recovery behaviors
- [ ] Dynamic reconfigure for runtime parameter tuning
