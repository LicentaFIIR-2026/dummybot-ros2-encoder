#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import struct

class DummyBotController(Node):
    def __init__(self):
        super().__init__('dummybot_controller')
        
        # Parametri
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.3)  # distanța între roți (m)
        self.declare_parameter('max_speed', 255)
        
        # Setup serial
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Așteaptă ESP32 să se inițializeze
            self.get_logger().info(f'Connected to ESP32 on {port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            return
        
        # Subscriber pentru comenzi de mișcare
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer pentru verificare conexiune
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.last_cmd_time = time.time()
        self.safety_timeout = 1.0  # Oprește robotul după 1s fără comenzi
        
        self.get_logger().info('DummyBot Controller started!')
    
    def cmd_vel_callback(self, msg):
        """Convertește Twist în comenzi pentru motoare"""
        linear = msg.linear.x   # m/s
        angular = msg.angular.z # rad/s
        
        # Calculează vitezele pentru roțile stânga și dreapta
        wheel_sep = self.get_parameter('wheel_separation').value
        max_speed = self.get_parameter('max_speed').value
        
        # Kinematică diferențială
        left_speed = linear - (angular * wheel_sep / 2.0)
        right_speed = linear + (angular * wheel_sep / 2.0)
        
        # Normalizează la [-255, 255]
        max_wheel_speed = max(abs(left_speed), abs(right_speed))
        if max_wheel_speed > 1.0:  # Presupunem viteza max = 1 m/s
            left_speed = left_speed / max_wheel_speed
            right_speed = right_speed / max_wheel_speed
        
        left_pwm = int(left_speed * max_speed)
        right_pwm = int(right_speed * max_speed)
        
        self.send_motor_command(left_pwm, right_pwm)
        self.last_cmd_time = time.time()
    
    def send_motor_command(self, left_speed, right_speed):
        """Trimite comanda către ESP32"""
        try:
            # Pachetul: 2 int-uri de 32 bit
            data = struct.pack('<ii', left_speed, right_speed)
            self.serial_conn.write(data)
            
            self.get_logger().debug(f'Sent: L={left_speed}, R={right_speed}')
            
        except Exception as e:
            self.get_logger().error(f'Serial send error: {e}')
    
    def timer_callback(self):
        """Verifică timeout-ul pentru siguranță"""
        if time.time() - self.last_cmd_time > self.safety_timeout:
            self.send_motor_command(0, 0)  # Oprește motoarele
    
    def __del__(self):
        if hasattr(self, 'serial_conn'):
            self.send_motor_command(0, 0)
            self.serial_conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = DummyBotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()