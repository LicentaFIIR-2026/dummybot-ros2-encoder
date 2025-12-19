#!/usr/bin/env python3
# lidar_direct_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from lidar_lib import lidar
import math

class LidarDirectNode(Node):
    def __init__(self):
        super().__init__('lidar_direct_node')
        
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Conectează direct la LiDAR
        self.lidar = lidar("/dev/ttyUSB1")
        if not self.lidar.open():
            self.get_logger().error("Cannot open LiDAR")
            return
        
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.get_logger().info('LiDAR direct node started')
    
    def publish_scan(self):
        measures = self.lidar.getMeasures()
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.1
        scan.range_max = 8.0
        
        ranges = [0.0] * 360
        for point in measures:
            if point.distance > 0:
                angle_index = int(point.angle)
                if 0 <= angle_index < 360:
                    ranges[angle_index] = point.distance / 1000.0
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDirectNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
