#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

class ScanBridge(Node):
    def __init__(self):
        super().__init__('scan_bridge')
        
        # Subscribe with RELIABLE
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publish with BEST_EFFORT
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_sub)
        self.publisher = self.create_publisher(
            LaserScan, '/scan_best_effort', qos_pub)
    
    def scan_callback(self, msg):
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = ScanBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
