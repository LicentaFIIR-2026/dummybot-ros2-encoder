#!/usr/bin/env python3
"""
Export toate waypoint-urile în format JSON pentru Claude
"""

import yaml
import json
import os
import math

WAYPOINTS_FILE = os.path.expanduser(
    '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml'
)

def quaternion_from_euler(yaw_deg):
    """Convertește yaw în grade în quaternion"""
    yaw = math.radians(yaw_deg)
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return {
        'x': 0.0,
        'y': 0.0,
        'z': round(qz, 6),
        'w': round(qw, 6)
    }

def export_all_waypoints():
    """Export toate waypoint-urile"""
    try:
        with open(WAYPOINTS_FILE, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'waypoints' in data:
                waypoints = data['waypoints']
                
                result = {}
                for name, wp in waypoints.items():
                    quat = quaternion_from_euler(wp['yaw'])
                    result[name] = {
                        'position': {
                            'x': round(wp['x'], 3),
                            'y': round(wp['y'], 3),
                            'z': 0.0
                        },
                        'orientation': quat,
                        'yaw_degrees': wp['yaw']
                    }
                
                return result
    except Exception as e:
        return {'error': str(e)}

if __name__ == '__main__':
    waypoints = export_all_waypoints()
    print(json.dumps(waypoints, indent=2))
