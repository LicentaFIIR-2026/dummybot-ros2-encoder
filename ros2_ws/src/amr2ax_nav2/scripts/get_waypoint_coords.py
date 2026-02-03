#!/usr/bin/env python3
"""
Helper script: returnează coordonatele unui waypoint în format JSON
Folosit de Claude pentru a naviga robotul
"""

import sys
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
        'z': qz,
        'w': qw
    }

def get_waypoint(name):
    """Returnează coordonatele unui waypoint"""
    try:
        with open(WAYPOINTS_FILE, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'waypoints' in data:
                waypoints = data['waypoints']
                
                if name in waypoints:
                    wp = waypoints[name]
                    quat = quaternion_from_euler(wp['yaw'])
                    
                    return {
                        'success': True,
                        'waypoint': name,
                        'position': {
                            'x': wp['x'],
                            'y': wp['y'],
                            'z': 0.0
                        },
                        'orientation': quat
                    }
                else:
                    return {
                        'success': False,
                        'error': f"Waypoint '{name}' nu există",
                        'available': list(waypoints.keys())
                    }
    except Exception as e:
        return {
            'success': False,
            'error': str(e)
        }

def list_waypoints():
    """Listează toate waypoint-urile"""
    try:
        with open(WAYPOINTS_FILE, 'r') as f:
            data = yaml.safe_load(f)
            if data and 'waypoints' in data:
                return {
                    'success': True,
                    'waypoints': data['waypoints']
                }
    except Exception as e:
        return {
            'success': False,
            'error': str(e)
        }

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(json.dumps(list_waypoints()))
    else:
        waypoint_name = sys.argv[1]
        print(json.dumps(get_waypoint(waypoint_name)))
