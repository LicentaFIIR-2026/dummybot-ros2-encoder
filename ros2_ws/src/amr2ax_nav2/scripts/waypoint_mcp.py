#!/usr/bin/env python3
"""
MCP Server pentru Waypoint Navigation
Claude se conectează direct la acest script
"""

import json
import sys
import os
import yaml
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math

# Import din scriptul original
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

WAYPOINTS_FILE = os.path.expanduser(
    '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml'
)

class WaypointMCPServer(Node):
    def __init__(self):
        super().__init__('waypoint_mcp_server')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.is_navigating = False
        self.waypoints = self.load_waypoints()
        
        # Start ROS spin în thread separat
        self.spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.spin_thread.start()
    
    def load_waypoints(self):
        """Încarcă waypoint-uri din YAML"""
        if os.path.exists(WAYPOINTS_FILE):
            try:
                with open(WAYPOINTS_FILE, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        return data['waypoints']
            except Exception as e:
                self.get_logger().warn(f'Eroare încărcare: {e}')
        return {}
    
    def spin_ros(self):
        """Thread pentru ROS2 spin"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convertește Euler în quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]
    
    def navigate_to(self, waypoint_name):
        """Navighează la waypoint"""
        if waypoint_name not in self.waypoints:
            return {
                "success": False,
                "error": f"Waypoint '{waypoint_name}' nu există. Disponibile: {list(self.waypoints.keys())}"
            }
        
        wp = self.waypoints[waypoint_name]
        x, y, yaw_deg = wp['x'], wp['y'], wp['yaw']
        yaw_rad = math.radians(yaw_deg)
        
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            return {
                "success": False,
                "error": "Nav2 nu răspunde! Verifică că navxplorer.launch.py rulează."
            }
        
        self.is_navigating = True
        
        # Creează goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        quat = self.quaternion_from_euler(0, 0, yaw_rad)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        # Trimite goal
        self._action_client.send_goal_async(goal_msg)
        
        return {
            "success": True,
            "message": f"Robotul navighează către '{waypoint_name}' la ({x:.2f}, {y:.2f}, {yaw_deg}°)"
        }

# MCP Protocol Handler
def send_response(response):
    """Trimite răspuns JSON pe stdout"""
    print(json.dumps(response), flush=True)

def handle_mcp_request(request, server):
    """Procesează cereri MCP"""
    method = request.get('method')
    request_id = request.get('id')
    
    if method == 'initialize':
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {
                    "name": "waypoint-navigation",
                    "version": "1.0.0"
                }
            }
        }
    
    elif method == 'tools/list':
        waypoints = list(server.waypoints.keys())
        return {
            "jsonrpc": "2.0",
            "id": request_id,
            "result": {
                "tools": [
                    {
                        "name": "navigate_robot",
                        "description": f"Navigate DummyBot to a waypoint. Available: {', '.join(waypoints)}",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "waypoint": {
                                    "type": "string",
                                    "enum": waypoints,
                                    "description": f"Waypoint name: {', '.join(waypoints)}"
                                }
                            },
                            "required": ["waypoint"]
                        }
                    },
                    {
                        "name": "list_waypoints",
                        "description": "List all saved waypoints with coordinates",
                        "inputSchema": {
                            "type": "object",
                            "properties": {}
                        }
                    }
                ]
            }
        }
    
    elif method == 'tools/call':
        tool_name = request['params']['name']
        arguments = request['params'].get('arguments', {})
        
        if tool_name == 'navigate_robot':
            waypoint = arguments.get('waypoint')
            result = server.navigate_to(waypoint)
            
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "content": [
                        {
                            "type": "text",
                            "text": json.dumps(result, indent=2)
                        }
                    ]
                }
            }
        
        elif tool_name == 'list_waypoints':
            waypoints_info = {}
            for name, wp in server.waypoints.items():
                waypoints_info[name] = f"({wp['x']:.2f}, {wp['y']:.2f}, {wp['yaw']:.1f}°)"
            
            return {
                "jsonrpc": "2.0",
                "id": request_id,
                "result": {
                    "content": [
                        {
                            "type": "text",
                            "text": json.dumps(waypoints_info, indent=2)
                        }
                    ]
                }
            }
    
    return {
        "jsonrpc": "2.0",
        "id": request_id,
        "error": {
            "code": -32601,
            "message": f"Method not found: {method}"
        }
    }

def main():
    # Inițializează ROS2
    rclpy.init()
    
    # Creează server-ul
    server = WaypointMCPServer()
    
    # Ascultă cereri MCP pe stdin
    try:
        for line in sys.stdin:
            if not line.strip():
                continue
            
            try:
                request = json.loads(line)
                response = handle_mcp_request(request, server)
                send_response(response)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                send_response({
                    "jsonrpc": "2.0",
                    "error": {
                        "code": -32603,
                        "message": str(e)
                    }
                })
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
