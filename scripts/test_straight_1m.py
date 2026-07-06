#!/usr/bin/env python3
"""
Test script for wheel_separation_multiplier calibration
Moves robot 1 meter forward and reports actual distance traveled
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import math
import time

class StraightLineTest(Node):
    def __init__(self):
        super().__init__('straight_line_test')
        
        # Publică direct pe topic-ul controller-ului
        self.cmd_pub = self.create_publisher(
            TwistStamped, 
            '/cmd_vel', 
            10
        )
        self.get_logger().info('Using TwistStamped for /diff_drive_controller/cmd_vel')
        
        # Subscribe la odometrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # State tracking
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.odom_received = False
        
        self.get_logger().info('✅ Node initialized. Waiting for odometry...')
    
    def odom_callback(self, msg):
        """Store current position"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('✅ Odometry received!')
    
    def send_velocity(self, linear_x):
        """Send velocity command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
    
    def stop_robot(self):
        """Stop robot"""
        self.send_velocity(0.0)
    
    def get_distance_traveled(self):
        """Calculate distance from start"""
        if self.start_x is None:
            return 0.0
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx*dx + dy*dy)
    
    def run_test(self, target_distance=1.0, velocity=0.2):
        """Run straight line test"""
        # Wait for odometry
        self.get_logger().info('⏳ Waiting for odometry data...')
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Record start position
        time.sleep(0.5)  # Stabilizare
        self.start_x = self.current_x
        self.start_y = self.current_y
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'🎯 TEST CALIBRARE WHEEL_SEPARATION_MULTIPLIER')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'📍 Start position: ({self.start_x:.3f}, {self.start_y:.3f})')
        self.get_logger().info(f'🎯 Target distance: {target_distance:.2f} m')
        self.get_logger().info(f'⚡ Velocity: {velocity:.2f} m/s')
        self.get_logger().info(f'{"="*60}\n')
        
        input('📏 MARCHEAZA POZITIA START PE PODEA! Press ENTER când ești gata...')
        
        # Start movement
        self.get_logger().info('🚀 Pornesc mișcarea...\n')
        start_time = time.time()
        last_print = 0.0
        
        while rclpy.ok():
            self.send_velocity(velocity)
            rclpy.spin_once(self, timeout_sec=0.01)
            
            distance = self.get_distance_traveled()
            
            # Progress la fiecare 10cm
            if distance - last_print >= 0.1:
                self.get_logger().info(f'📍 {distance:.2f} m')
                last_print = distance
            
            if distance >= target_distance:
                break
        
        # Stop
        self.stop_robot()
        time.sleep(0.2)
        
        final_distance_odom = self.get_distance_traveled()
        final_time = time.time() - start_time
        
        # Așteaptă măsurătoarea manuală
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'🛑 ROBOT OPRIT!')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'⏱️  Timp: {final_time:.2f} s')
        self.get_logger().info(f'📊 Distanță raportată (odometrie): {final_distance_odom:.3f} m')
        self.get_logger().info(f'{"="*60}\n')
        
        try:
            measured = input('📏 Măsoară cu RULETA distanța reală și introdu (ex: 0.95): ')
            measured_distance = float(measured)
            
            # Calculate error
            error_m = measured_distance - target_distance
            error_percent = (error_m / target_distance) * 100
            
            # Results
            self.get_logger().info(f'\n{"="*60}')
            self.get_logger().info(f'📊 REZULTATE FINALE:')
            self.get_logger().info(f'{"="*60}')
            self.get_logger().info(f'🎯 Target:           {target_distance:.3f} m')
            self.get_logger().info(f'📏 Măsurat (ruletă): {measured_distance:.3f} m')
            self.get_logger().info(f'📊 Raportat (odom):  {final_distance_odom:.3f} m')
            self.get_logger().info(f'❌ Eroare reală:     {error_m:+.3f} m ({error_percent:+.1f}%)')
            self.get_logger().info(f'{"="*60}\n')
            
            # Calibration suggestion
            if abs(error_percent) > 2.0:
                current_multiplier = 1.0
                # Calcul: dacă robotul merge prea mult → wheel_separation prea mic → mărește multiplier
                suggested_multiplier = current_multiplier * (measured_distance / target_distance)
                
                self.get_logger().info(f'💡 SUGESTIE CALIBRARE:')
                self.get_logger().info(f'{"="*60}')
                self.get_logger().info(f'Curent wheel_separation_multiplier: {current_multiplier}')
                self.get_logger().info(f'Sugerat nou multiplier: {suggested_multiplier:.4f}')
                self.get_logger().info(f'\n📝 Modifică în dummybot_controllers.yaml:')
                self.get_logger().info(f'   wheel_separation_multiplier: {suggested_multiplier:.4f}')
                self.get_logger().info(f'{"="*60}\n')
            else:
                self.get_logger().info(f'✅ Calibrare EXCELENTĂ! (eroare < 2%)\n')
                
        except ValueError:
            self.get_logger().error('❌ Valoare invalidă introdusă!')
        except KeyboardInterrupt:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineTest()
    
    try:
        node.run_test(target_distance=1.0, velocity=0.2)
        
        # Keep stopping
        for _ in range(10):
            node.stop_robot()
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Test întrerupt!')
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
