# DummyBot Autonomous Navigation

Autonomous navigation system using Nav2 stack with AMCL localization and dynamic obstacle avoidance.

## System Overview

**Hardware:**
- 4WD skid-steer robot (normal wheels, not mecanum)
- LD06 LiDAR (12m range, 360°, 10Hz)
- ESP32 motor control with Hall AB dual-phase encoders (1760 ticks/rev)
- Raspberry Pi 5 (Ubuntu 24.04, ROS2 Jazzy)

**Navigation Stack:**
- **AMCL**: Adaptive Monte Carlo Localization with OmniMotionModel for skid-steer
- **NavFn Planner**: Global path planning (A* algorithm)
- **DWB Controller**: Dynamic Window Approach for local planning
- **Costmaps**: Static map + dynamic obstacle detection via LiDAR

---

## Hardware Calibration

### Encoder Calibration

**Critical for accurate odometry!** Encoders were calibrated using ESP32 command `c` to ensure equal motor performance:
```bash
# ESP32 Serial Monitor:
c 0.875 1.065 1.025 1.025

# Resulting tick counts after calibration (test: m 100 100 100 100):
# FL: 2849  FR: 2847  RL: 2849  RR: 2849
# Difference: ±5 ticks (0.2%) - Excellent!
```

**FR Encoder Fix**: Front Right encoder channels A and B were **swapped** because it initially reported negative values in `/joint_states`. After swapping, all encoders report positive rotation for forward motion.

### Robot Geometry (Measured)

- **Wheel diameter**: 69mm (radius: 34.5mm)
- **Wheel width**: 28mm
- **Track width** (lateral, X-axis): 374mm (center-to-center)
- **Wheelbase** (longitudinal, Y-axis): 390mm (center-to-center)
- **Chassis**: 460mm (length) × 360mm (width) × 140mm (height)
- **Ground clearance**: 50mm (soil to chassis base)
- **LiDAR position**: 10cm forward of center, 17.5cm above chassis base

### URDF Configuration

**Key modification**: Robot rotated **-90° on Z-axis** in `base_footprint_joint` to align robot's physical front with ROS2 convention (X-axis = forward).
```xml
<!-- dummybot.urdf.xacro -->
<origin xyz="0 0 ${ground_clearance}" rpy="0 0 ${-pi/2}"/>
```

**Result**: Green arrow (Y-axis) now points forward in physical robot orientation.

---

## Quick Start

### 1. Launch Navigation System

**Terminal 1 - Scan Bridge (QoS compatibility):**
```bash
python3 ~/dummybot-ros2-encoder/scripts/scan_bridge.py
```
*Purpose: Converts LiDAR scan from RELIABLE to BEST_EFFORT QoS for AMCL compatibility*

**Terminal 2 - Navigation Stack (wait 15 seconds after T1):**
```bash
cd ~/dummybot-ros2-encoder/ros2_ws
source install/setup.bash
ros2 launch dummybot_navigation navigation.launch.py
```
*Launches: robot hardware, LiDAR, controllers, AMCL, Nav2 planning/control*

**Terminal 3 - Cmd Vel Bridge:**
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
   - Map → Topic: `/local_costmap/costmap` (local obstacle detection)
   - RobotModel
   - PoseArray → Topic: `/particlecloud` (AMCL particles)

3. **Set goal:**
   - Toolbar: Click **"Nav2 Goal"** or **"2D Goal Pose"**
   - Click on map (white/free space)
   - Drag to set orientation
   - **Robot will navigate autonomously!**

---

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

Odometry:
ESP32 encoders → hardware_interface → /diff_drive_controller/odom → AMCL + Nav2
```

### TF Tree
```
map → odom → base_footprint → base_link → laser_frame
                                        → front_left_wheel_link
                                        → front_right_wheel_link
                                        → rear_left_wheel_link
                                        → rear_right_wheel_link
```

**Critical**: `base_frame_id` for diff_drive_controller is `base_footprint` (NOT `base_link`) to ensure proper odometry integration with AMCL.

---

## Navigation Configuration (nav2_params.yaml)

### AMCL Tuning for Skid-Steer

**Motion Model**: `OmniMotionModel` (instead of DifferentialMotionModel) because 4WD skid-steer robots can drift laterally during rotation.

**Key Parameters:**
```yaml
robot_model_type: "nav2_amcl::OmniMotionModel"
alpha1: 0.4  # Rotation → rotation noise (increased for drift tolerance)
alpha2: 0.4  # Translation → rotation (lateral drift during forward motion)
alpha3: 0.4  # Translation → translation
alpha4: 0.4  # Rotation → translation (forward drift during rotation)
```

**Why alpha=0.4 instead of 0.2?** Skid-steer robots slip laterally when rotating, causing drift. Higher alpha values tell AMCL to expect more noise, making localization more robust.

**Recovery Behaviors**: Enabled to handle "kidnapping" scenarios
```yaml
recovery_alpha_slow: 0.001  # Was 0.0 (disabled)
recovery_alpha_fast: 0.1    # Was 0.0 (disabled)
```

**Update Frequency**: Increased for faster drift correction
```yaml
update_min_d: 0.15  # Update every 15cm (was 25cm)
update_min_a: 0.15  # Update every ~8° (was ~11°)
```

### Controller Configuration

**Goal Tolerances**: Relaxed for skid-steer drift
```yaml
xy_goal_tolerance: 0.25   # 25cm (was 15cm)
yaw_goal_tolerance: 0.40  # ~23° (was ~9°)
```

**Velocity Limits**: Balanced for control vs speed
```yaml
max_vel_x: 1.0 m/s
max_vel_theta: 2.0 rad/s
trans_stopped_velocity: 0.10  # Slow down near goal (was 0.25)
```

**Obstacle Avoidance**: Increased weight
```yaml
BaseObstacle.scale: 0.05  # Was 0.02 (2.5x more aggressive)
```

### Local Costmap

**Layer**: Changed from `voxel_layer` to `obstacle_layer` for simpler, faster processing.

**Update Rate**: Increased for real-time obstacle detection
```yaml
update_frequency: 10.0  # 10Hz (was 5Hz)
publish_frequency: 5.0  # 5Hz (was 2Hz)
```

**Robot Size**: Conservative for safety
```yaml
robot_radius: 0.35m  # Was 0.28m (added 7cm safety margin)
```

**Inflation**: More aggressive obstacle avoidance
```yaml
inflation_radius: 0.55m
cost_scaling_factor: 5.0  # Was 3.0
```

**Frame Configuration:**
```yaml
robot_base_frame: base_footprint  # Was base_link
scan.sensor_frame: laser_frame
scan.topic: /scan_best_effort
```

---

## Performance & Known Issues

### Success Rate

**~70-80%** goal completion in typical indoor environments

**Successful scenarios:**
- Straight-line navigation
- Simple obstacle avoidance
- Goal-to-goal waypoints in open spaces

**Challenging scenarios:**
- Tight corners with <50cm clearance
- Very narrow passages
- Complex multi-obstacle scenarios

### Known Issues

1. **Occasional "Goal Failed"** (~20-30% of attempts)
   - **Cause**: Skid-steer drift accumulation + tight tolerances in confined spaces
   - **Mitigation**: Goal tolerances already relaxed (xy=25cm, yaw=23°)
   - **Workaround**: Retry goal or choose intermediate waypoint

2. **Slow rotation near goal**
   - **Cause**: DWB planner recalculates frequently when trying to achieve precise orientation
   - **Mitigation**: `trans_stopped_velocity` lowered + `RotateToGoal.slowing_factor` increased
   - **Status**: Improved but not eliminated

3. **Drift during rotation**
   - **Cause**: Inherent to skid-steer kinematics (wheels slip laterally)
   - **Mitigation**: AMCL OmniMotionModel + encoder calibration
   - **Result**: 3-5cm lateral drift per 360° rotation (acceptable)

---

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

**"Failed to transform"**: TF timing issue, usually resolves after initial pose set. If persistent, check `transform_tolerance` in nav2_params.yaml.

**"Message Filter dropping message: frame 'base_laser'"**: Frame mismatch between LiDAR driver and AMCL config. Verify `laser_frame_id` matches LiDAR's published frame.

### Navigation fails

1. **Goal in occupied space**: Choose goal in white (free) area on map
2. **Path blocked**: Robot will replan automatically
3. **"Failed to make progress"**: Goal may be unreachable, try closer goal or increase `movement_time_allowance`
4. **Robot rotates endlessly near goal**: Yaw tolerance too strict, already relaxed to 0.40 rad

### Map doesn't appear in RViz

1. **Check Map display**: Durability Policy must be `Transient Local`
2. **Verify map server**:
```bash
   ros2 topic echo /map --once
```

### Encoders report incorrect values

1. **Check calibration**: Run `m 100 100 100 100` in ESP32, all wheels should report similar tick counts
2. **Negative velocities**: Swap encoder channels A↔B in hardware or invert in software
3. **Asymmetric drift**: Recalibrate with command `c` in ESP32

---

## Files

- `config/nav2_params.yaml`: Complete Nav2 configuration (AMCL, costmaps, controller)
- `config/dummybot_controllers.yaml`: ros2_control diff_drive configuration
- `description/urdf/dummybot.urdf.xacro`: Robot URDF with -90° Z rotation
- `launch/navigation.launch.py`: Main navigation launch file
- `maps/my_map.yaml` + `my_map.pgm`: Saved map from SLAM
- `../../../scripts/scan_bridge.py`: LiDAR QoS bridge
- `../../../scripts/utils/cmd_vel_bridge.py`: Cmd_vel type converter

---

## Future Improvements

### High Priority
- [ ] Integrate bridges into launch files (eliminate manual terminal startup)
- [ ] Auto initial pose (save/restore last known position)
- [ ] Waypoint navigation (multi-goal sequences)

### Medium Priority
- [ ] Fine-tune DWB local planner (reduce "thinking time" near goals)
- [ ] Recovery behaviors (backup, rotate, clear costmaps)
- [ ] Dynamic reconfigure for runtime parameter tuning
- [ ] RViz config file (persistent display setup)

### Low Priority
- [ ] Multi-floor/multi-map support
- [ ] Global localization (for "kidnapped robot" scenarios)
- [ ] Behavior tree customization for complex tasks

---

## Version History

**v1.0** (Current)
- Encoder calibration: ±5 ticks accuracy (0.2%)
- URDF: -90° Z rotation for proper orientation
- AMCL: OmniMotionModel with alpha=0.4
- Navigation: ~70-80% success rate
- Known issues: Occasional goal failures in tight spaces

---

## Credits

**Hardware**: DummyBot 4WD platform with LD06 LiDAR
**Software**: ROS2 Jazzy, Nav2, SLAM Toolbox
**Development**: Andrei @ Politehnica București (UPB-FIIR)
**License**: [Your License]