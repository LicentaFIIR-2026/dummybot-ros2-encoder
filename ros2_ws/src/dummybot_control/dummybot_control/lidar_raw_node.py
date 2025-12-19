#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import math
import time

class LidarRawNode(Node):
    def __init__(self):
        super().__init__('lidar_raw_node')
        
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Conectează direct la LiDAR prin USB-UART
        try:
            self.serial_conn = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info('LiDAR connected directly on /dev/ttyUSB1')
        except Exception as e:
            self.get_logger().error(f'Failed to connect LiDAR: {e}')
            return
        
        # Buffer pentru pachete
        self.packet_buffer = []
        self.scan_points = {}
        
        # Timers - mai rapide pentru mai multe puncte
        self.read_timer = self.create_timer(0.005, self.read_lidar_data)  # 200Hz
        self.scan_timer = self.create_timer(0.05, self.publish_scan)  # 20Hz
        
        self.get_logger().info('LiDAR raw node started!')
    
    def read_lidar_data(self):
        """Citește și procesează date raw de la LiDAR"""
        try:
            while self.serial_conn.in_waiting:
                byte = self.serial_conn.read(1)[0]
                self.packet_buffer.append(byte)
                
                # Procesează la fiecare 7 bytes în loc să cauți doar 0x5A
                if len(self.packet_buffer) >= 7:
                    self.try_parse_any_packet()
                
                # Previne overflow
                if len(self.packet_buffer) > 100:
                    self.packet_buffer = self.packet_buffer[-50:]
                    
        except Exception as e:
            self.get_logger().warn(f'LiDAR read error: {e}')
    
    def try_parse_any_packet(self):
        """Încearcă să parseze orice pattern din buffer"""
        for i in range(len(self.packet_buffer) - 6):
            if self.try_parse_at_position(i):
                # Găsit packet valid, elimină bytes procesați
                self.packet_buffer = self.packet_buffer[i+7:]
                return
        
        # Dacă nu găsește nimic valid, elimină primul byte
        if len(self.packet_buffer) > 20:
            self.packet_buffer.pop(0)

    def try_parse_at_position(self, start):
        """Încearcă parsarea începând de la poziția dată"""
        if start + 6 >= len(self.packet_buffer):
            return False
        
        # Încercă diferite combinații de bytes pentru distanță și unghi
        distance1 = self.packet_buffer[start] | (self.packet_buffer[start+1] << 8)
        angle1 = self.packet_buffer[start+2] | (self.packet_buffer[start+3] << 8)
        
        # Verifică primul format
        if self.is_valid_measurement(distance1, angle1):
            self.add_point(distance1, angle1)
            return True
        
        # Încearcă format inversat
        distance2 = self.packet_buffer[start+1] | (self.packet_buffer[start] << 8)
        angle2 = self.packet_buffer[start+3] | (self.packet_buffer[start+2] << 8)
        
        if self.is_valid_measurement(distance2, angle2):
            self.add_point(distance2, angle2)
            return True
        
        # Încearcă cu offset diferit
        if start + 5 < len(self.packet_buffer):
            distance3 = self.packet_buffer[start+1] | (self.packet_buffer[start+2] << 8)
            angle3 = self.packet_buffer[start+3] | (self.packet_buffer[start+4] << 8)
            
            if self.is_valid_measurement(distance3, angle3):
                self.add_point(distance3, angle3)
                return True
        
        return False
    
    def is_valid_measurement(self, distance, angle):
        """Verifică dacă măsurătoarea e validă"""
        # Distanță între 5cm și 8m
        if not (50 <= distance <= 8000):
            return False
        
        # Unghi între 0 și 36000 (sute de grade) sau 0-360 (grade)
        if angle <= 360:
            return True
        elif 0 <= angle <= 36000:
            return True
        
        return False
    
    def add_point(self, distance, angle):
        """Adaugă punct valid la scan"""
        distance_m = distance / 1000.0
        
        # Convertește unghiul la grade
        if angle > 360:
            angle_deg = (angle / 100.0) % 360.0
        else:
            angle_deg = angle % 360.0
        
        angle_index = int(angle_deg)
        self.scan_points[angle_index] = distance_m
    
    def publish_scan(self):
        """Publică LaserScan"""
        if len(self.scan_points) < 1:  # Publică orice punct
            return
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.05
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
    node = LidarRawNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()