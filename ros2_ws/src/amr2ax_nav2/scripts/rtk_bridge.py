#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import json
import threading

class RTKBridge(Node):
    def __init__(self):
        super().__init__('rtk_bridge')
        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        
        # Conectare la ESP32 prin USB
        try:
            self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info('Connected to ESP32 RTK rover on /dev/ttyUSB1')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return
        
        # Thread pentru citire serial
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()
        
        self.get_logger().info('RTK Bridge active - publishing to /fix at ~13Hz')
    
    def read_loop(self):
        while rclpy.ok() and self.running:
            try:
                line = self.serial.readline().decode().strip()
                if line.startswith('{"header":') and '"gps_link"' in line:
                    data = json.loads(line)
                    
                    msg = NavSatFix()
                    # Use ROS time for timestamp to ensure synchronization
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps_link'
                    
                    msg.status.status = data['status']['status']
                    msg.status.service = data['status']['service']
                    
                    msg.latitude = data['latitude']
                    msg.longitude = data['longitude']
                    msg.altitude = data['altitude']
                    msg.position_covariance = data['position_covariance']
                    msg.position_covariance_type = 2
                    
                    self.pub.publish(msg)
                    
            except json.JSONDecodeError:
                pass
            except Exception as e:
                self.get_logger().debug(f'Read error: {e}')
    
    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial'):
            self.serial.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = RTKBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()