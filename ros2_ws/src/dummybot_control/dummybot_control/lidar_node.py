#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import time

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Parametri
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        
        port = self.get_parameter('serial_port').value
        
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f'LiDAR connected on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect LiDAR: {e}')
            return
        
        # Buffer pentru pachete LiDAR
        self.packet_buffer = []
        self.scan_points = {}
        
        self.timer = self.create_timer(0.01, self.read_lidar_data)
        self.scan_timer = self.create_timer(0.2, self.publish_scan)  # 5Hz
        
        self.get_logger().info('LiDAR node started!')
    
    def read_lidar_data(self):
        """Citește pachete LiDAR"""
        try:
            while self.serial_conn.in_waiting:
                byte = self.serial_conn.read(1)[0]
                self.packet_buffer.append(byte)
                
                # Caută pattern de packet LiDAR (0x5A header)
                if byte == 0x5A and len(self.packet_buffer) > 1:
                    self.process_packet()
                    self.packet_buffer = [0x5A]  # Începe noul packet
                
                # Previne overflow
                if len(self.packet_buffer) > 50:
                    self.packet_buffer = []
                        
        except Exception as e:
            self.get_logger().warn(f'LiDAR read error: {e}')
    
    def process_packet(self):
        """Procesează un pachet LiDAR complet"""
        if len(self.packet_buffer) >= 7:
            # Încearcă să parsezi: [0x5A] [dist_L] [dist_H] [angle_L] [angle_H] [quality] [flags]
            distance = self.packet_buffer[1] | (self.packet_buffer[2] << 8)
            angle = self.packet_buffer[3] | (self.packet_buffer[4] << 8)
            
            # Validări
            if 100 <= distance <= 8000:  # 10cm - 8m
                distance_m = distance / 1000.0
                angle_deg = (angle / 100.0) % 360.0
                
                angle_index = int(angle_deg)
                self.scan_points[angle_index] = distance_m
    
    def publish_scan(self):
        """Publică LaserScan"""
        if len(self.scan_points) < 10:
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.1
        scan.range_max = 8.0
        
        ranges = [0.0] * 360
        for angle_deg, distance in self.scan_points.items():
            if 0 <= angle_deg < 360:
                ranges[angle_deg] = distance
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)
        
        self.get_logger().info(f'Published scan: {len(self.scan_points)} points')
        self.scan_points.clear()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()