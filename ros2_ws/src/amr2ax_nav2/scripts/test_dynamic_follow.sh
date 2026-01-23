#!/bin/bash
#
# Script rapid de testare pentru Dynamic Object Following
# 
# Usage: ./test_dynamic_follow.sh
#

echo "========================================="
echo "Dynamic Object Following - Quick Test"
echo "========================================="
echo ""

# Check dacă workspace-ul e source'd
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS environment not sourced!"
    echo "Run: source ~/saim_xplorer/install/setup.bash"
    exit 1
fi

echo "✓ ROS environment: $ROS_DISTRO"

# Check dacă pachetul e compilat
if ! ros2 pkg list | grep -q "amr2ax_nav2"; then
    echo "ERROR: Package amr2ax_nav2 not found!"
    echo "Run: colcon build --packages-select amr2ax_nav2"
    exit 1
fi

echo "✓ Package amr2ax_nav2 found"

# Check dacă nodurile necesare rulează
echo ""
echo "Checking running nodes..."

if ! ros2 node list | grep -q "bt_navigator"; then
    echo "WARNING: bt_navigator not running!"
    echo "Please start Nav2 first:"
    echo "  ros2 launch amr2ax_nav2 navxplorer.launch.py localization_type:=2D slam:=False use_sim_time:=False"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "✓ bt_navigator is running"
fi

# Check topicuri
echo ""
echo "Checking topics..."

if ros2 topic list | grep -q "/clicked_point"; then
    echo "✓ /clicked_point topic available"
else
    echo "WARNING: /clicked_point not available (RViz not running?)"
fi

# Pornește nodul convertor
echo ""
echo "========================================="
echo "Starting clicked_point_to_goal_update..."
echo "========================================="
echo ""

ros2 launch amr2ax_nav2 test_dynamic_following.launch.py
