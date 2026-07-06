#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
import time

# ============================================
# PARAMETRI CONFIGURABILI
# ============================================
TARGET_ROTATION_DEG = 90.0  # Grade de rotație (+90, -90, 180, -180, etc.)
ANGULAR_VELOCITY = 0.5      # Viteză rotație (rad/s) - MODIFICĂ pentru mai rapid/încet
TOLERANCE_DEG = 2.0         # Toleranță în grade (±2°)
# ============================================

class RotationTest(Node):
    def __init__(self):
        super().__init__('rotation_test')
        
        # Convert target to radians
        self.target_rotation_rad = math.radians(TARGET_ROTATION_DEG)
        self.tolerance_rad = math.radians(TOLERANCE_DEG)
        
        # Publisher pentru comenzi - direct pe /cmd_vel
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        
        # Subscriber pentru odometrie
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # State variables
        self.current_yaw = None
        self.initial_yaw = None
        self.rotation_complete = False
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROTATION TEST NODE STARTED')
        self.get_logger().info(f'Target: {TARGET_ROTATION_DEG}° ({self.target_rotation_rad:.3f} rad)')
        self.get_logger().info(f'Angular velocity: {ANGULAR_VELOCITY} rad/s')
        self.get_logger().info(f'Tolerance: ±{TOLERANCE_DEG}°')
        self.get_logger().info('=' * 60)
        
    def odom_callback(self, msg):
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def run_rotation(self):
        # Wait for first odometry message
        self.get_logger().info('Waiting for odometry on /odom...')
        timeout_counter = 0
        while self.current_yaw is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_counter += 1
            if timeout_counter > 50:  # 5 secunde
                self.get_logger().error('TIMEOUT: Nu primesc odometrie pe /odom!')
                self.get_logger().error('Verifică: ros2 topic list | grep odom')
                self.get_logger().error('Verifică: ros2 topic hz /odom')
                return
        
        # Store initial yaw
        self.initial_yaw = self.current_yaw
        self.get_logger().info(f'✓ Odometry received!')
        self.get_logger().info(f'Initial yaw: {math.degrees(self.initial_yaw):.2f}°')
        
        # Wait 2 seconds
        self.get_logger().info('Starting rotation in 2 seconds...')
        time.sleep(2.0)
        
        # Create velocity command
        cmd = TwistStamped()
        cmd.header.frame_id = 'base_link'
        
        if self.target_rotation_rad > 0:
            cmd.twist.angular.z = ANGULAR_VELOCITY
        else:
            cmd.twist.angular.z = -ANGULAR_VELOCITY
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'STARTING ROTATION {"LEFT" if cmd.twist.angular.z > 0 else "RIGHT"}')
        self.get_logger().info(f'Publishing angular.z = {cmd.twist.angular.z} rad/s on /cmd_vel')
        self.get_logger().info('=' * 60)
        
        # Rotation loop
        rate = self.create_rate(20)  # 20 Hz
        loop_count = 0
        max_loops = 400  # 20 secunde maximum (20Hz * 20s)
        
        while rclpy.ok() and not self.rotation_complete and loop_count < max_loops:
            # Update timestamp
            cmd.header.stamp = self.get_clock().now().to_msg()
            
            # Publish command
            self.cmd_pub.publish(cmd)
            
            # Calculate rotation from start
            rotation_delta = self.normalize_angle(self.current_yaw - self.initial_yaw)
            
            # Log progress more frequently
            if loop_count % 10 == 0:  # La fiecare 0.5 secunde
                self.get_logger().info(
                    f'Progress: {math.degrees(rotation_delta):6.2f}° / {TARGET_ROTATION_DEG}° '
                    f'| Current yaw: {math.degrees(self.current_yaw):6.2f}° '
                    f'| Error: {abs(math.degrees(rotation_delta) - TARGET_ROTATION_DEG):5.2f}°'
                )
            
            # Check if target reached
            error = abs(rotation_delta - self.target_rotation_rad)
            if error < self.tolerance_rad:
                self.rotation_complete = True
                self.get_logger().info('✓ Target reached!')
                break
            
            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()
            loop_count += 1
        
        if loop_count >= max_loops:
            self.get_logger().warn('⚠ TIMEOUT: Rotația nu s-a completat în 20 secunde!')
        
        # Stop robot
        self.get_logger().info('Stopping robot...')
        stop_cmd = TwistStamped()
        stop_cmd.header.frame_id = 'base_link'
        for _ in range(10):
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(stop_cmd)
            time.sleep(0.05)
        
        # Final report
        final_rotation = self.normalize_angle(self.current_yaw - self.initial_yaw)
        error_deg = abs(math.degrees(final_rotation) - TARGET_ROTATION_DEG)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ROTATION TEST COMPLETE')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Target rotation:  {TARGET_ROTATION_DEG:6.2f}°')
        self.get_logger().info(f'Actual rotation:  {math.degrees(final_rotation):6.2f}°')
        self.get_logger().info(f'Error:            {error_deg:6.2f}°')
        self.get_logger().info(f'Initial yaw:      {math.degrees(self.initial_yaw):6.2f}°')
        self.get_logger().info(f'Final yaw:        {math.degrees(self.current_yaw):6.2f}°')
        if error_deg < TOLERANCE_DEG:
            self.get_logger().info('✓ SUCCESS: Within tolerance!')
        else:
            self.get_logger().warn(f'⚠ WARNING: Error exceeds tolerance (±{TOLERANCE_DEG}°)')
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = RotationTest()
    
    try:
        node.run_rotation()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        # Stop robot
        node.get_logger().info('Emergency stop...')
        stop_cmd = TwistStamped()
        stop_cmd.header.frame_id = 'base_link'
        for _ in range(10):
            stop_cmd.header.stamp = node.get_clock().now().to_msg()
            node.cmd_pub.publish(stop_cmd)
            time.sleep(0.05)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()