#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import serial
import struct
import math
import time

class UnifiedController(Node):
    def __init__(self):
        super().__init__('unified_controller')
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Parametri
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('wheel_separation', 0.3)
        self.declare_parameter('max_speed', 200)
        
        # Setup serial
        port = self.get_parameter('serial_port').value
        
        try:
            self.serial_conn = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f'Unified controller connected on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return
        
        # LiDAR data
        self.packet_buffer = []
        self.scan_points = {}
        
        # Timers
        self.read_timer = self.create_timer(0.01, self.read_serial_data)
        self.scan_timer = self.create_timer(0.01, self.publish_scan)
        
        # Safety
        self.last_cmd_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info('Unified controller started!')
    
    def cmd_vel_callback(self, msg):
        """Procesează comenzi motoare"""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Kinematică diferențială
        wheel_sep = self.get_parameter('wheel_separation').value
        max_speed = self.get_parameter('max_speed').value
        
        left_speed = linear - (angular * wheel_sep / 2.0)
        right_speed = linear + (angular * wheel_sep / 2.0)
        
        # Normalizare
        max_wheel_speed = max(abs(left_speed), abs(right_speed))
        if max_wheel_speed > 1.0:
            left_speed = left_speed / max_wheel_speed
            right_speed = right_speed / max_wheel_speed
        
        left_pwm = int(left_speed * max_speed)
        right_pwm = int(right_speed * max_speed)
        
        self.send_motor_command(left_pwm, right_pwm)
        self.last_cmd_time = time.time()
    
    def send_motor_command(self, left_speed, right_speed):
        """Trimite comandă motoare către ESP32"""
        try:
            data = struct.pack('<ii', left_speed, right_speed)
            self.serial_conn.write(data)
        except Exception as e:
            self.get_logger().error(f'Motor command error: {e}')
    
    def read_serial_data(self):
        """Citește toate datele de la ESP32"""
        try:
            # Dacă sunt comenzi motoare (8 bytes), le ignorăm aici
            # Restul sunt date LiDAR
            while self.serial_conn.in_waiting:
                byte = self.serial_conn.read(1)[0]
                self.process_lidar_byte(byte)
                
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
    
    def process_lidar_byte(self, byte):
        """Procesează bytes LiDAR"""
        self.packet_buffer.append(byte)
        
        # Caută header 0x5A
        if byte == 0x5A and len(self.packet_buffer) > 1:
            if len(self.packet_buffer) >= 7:
                self.parse_lidar_packet()
            self.packet_buffer = [0x5A]
        
        # Previne overflow
        if len(self.packet_buffer) > 50:
            self.packet_buffer = []
    
    def parse_lidar_packet(self):
        """Parsează pachet LiDAR complet"""
        if len(self.packet_buffer) >= 7:
            distance = self.packet_buffer[1] | (self.packet_buffer[2] << 8)
            angle = self.packet_buffer[3] | (self.packet_buffer[4] << 8)
            
            if 100 <= distance <= 8000:
                distance_m = distance / 1000.0
                angle_deg = (angle / 100.0) % 360.0
                
                angle_index = int(angle_deg)
                self.scan_points[angle_index] = distance_m
    
    def publish_scan(self):
        """Publică LaserScan"""
        if len(self.scan_points) < 0.5:
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
        
        self.get_logger().info(f'Scan: {len(self.scan_points)} points')
        self.scan_points.clear()
    
    def safety_check(self):
        """Oprește motoarele dacă nu vin comenzi"""
        if time.time() - self.last_cmd_time > 1.0:
            self.send_motor_command(0, 0)

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
