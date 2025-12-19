#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from media_pipe_ros2_msg.msg import MediaPipeHumanPoseList
import sys
import select
import termios
import tty

class PoseFollower(Node):
    def __init__(self):
        super().__init__('pose_follower')
        
        # Parametri - viteze și distanțe
        self.max_linear_speed = 18.0   # m/s
        self.max_angular_speed = 17.0  # rad/s
        self.speed_increment = 0.05
        
        # Distanță țintă (bazat pe înălțimea corpului în cadru)
        self.target_body_height = 0.4  # ~40% din cadru = distanță bună
        self.height_tolerance = 0.05   # Toleranță ±5%
        
        # Zone de control
        self.declare_parameter('center_tolerance', 0.15)
        self.declare_parameter('safety_timeout', 1.0)
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            MediaPipeHumanPoseList,
            '/mediapipe/human_pose_list',
            self.pose_callback,
            10
        )
        
        # Safety
        self.last_detection_time = self.get_clock().now()
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.timer = self.create_timer(0.1, self.safety_check)
        
        # Keyboard
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
    
    def print_instructions(self):
        msg = """
╔══════════════════════════════════════════════════════════╗
║           DummyBot Pose Follower Control                 ║
╚══════════════════════════════════════════════════════════╝

📋 COMENZI TASTATURĂ:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  w/x : Crește/Scade viteza liniară cu 0.05 m/s
  a/d : Crește/Scade viteza angulară cu 0.1 rad/s
  
  r/f : Crește/Scade distanța țintă cu 0.02
  
  s   : STOP forțat
  q/z : ±10% AMBELE viteze
  i   : Afișează setări curente
  h   : Afișează meniu
  
  CTRL+C : Ieșire
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🤖 COMPORTAMENT:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Persoană STÂNGA     → Robot rotește STÂNGA
  Persoană DREAPTA    → Robot rotește DREAPTA
  
  Persoană APROAPE    → Robot merge ÎNAPOI
  Persoană DEPARTE    → Robot merge ÎNAINTE
  Distanță OPTIMĂ     → Robot stă pe loc
  
  Fără persoană       → Robot se OPREȘTE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
        print(msg)
        self.print_current_settings()
    
    def print_current_settings(self):
        print(f"⚙️  Linear: {self.max_linear_speed:.2f} m/s | Angular: {self.max_angular_speed:.2f} rad/s | "
              f"Target distance: {self.target_body_height:.2f}")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def process_keyboard(self):
        key = self.get_key()
        
        if key == 'w':
            self.max_linear_speed += self.speed_increment
            print(f"⬆️  Linear: {self.max_linear_speed:.2f} m/s")
        elif key == 'x':
            self.max_linear_speed = max(0.0, self.max_linear_speed - self.speed_increment)
            print(f"⬇️  Linear: {self.max_linear_speed:.2f} m/s")
        elif key == 'a':
            self.max_angular_speed += 0.1
            print(f"⬆️  Angular: {self.max_angular_speed:.2f} rad/s")
        elif key == 'd':
            self.max_angular_speed = max(0.0, self.max_angular_speed - 0.1)
            print(f"⬇️  Angular: {self.max_angular_speed:.2f} rad/s")
        elif key == 'r':
            self.target_body_height += 0.02
            print(f"📏 Distanță țintă: {self.target_body_height:.2f} (mai departe)")
        elif key == 'f':
            self.target_body_height = max(0.1, self.target_body_height - 0.02)
            print(f"📏 Distanță țintă: {self.target_body_height:.2f} (mai aproape)")
        elif key == 'q':
            self.max_linear_speed *= 1.1
            self.max_angular_speed *= 1.1
            print(f"📈 +10%")
        elif key == 'z':
            self.max_linear_speed *= 0.9
            self.max_angular_speed *= 0.9
            print(f"📉 -10%")
        elif key == 's':
            self.stop_robot()
            print("🛑 STOP!")
        elif key == 'i':
            self.print_current_settings()
        elif key == 'h':
            self.print_instructions()
        elif key == '\x03':
            raise KeyboardInterrupt
    
    def pose_callback(self, msg):
        if msg.num_humans == 0:
            self.stop_robot()
            return
        
        pose = msg.human_pose
        
        # Update timing
        self.last_detection_time = self.get_clock().now()
        
        # Control bazat pe poziție
        self.position_control(pose)
    
    def position_control(self, pose):
        twist = Twist()
        
        center_tol = self.get_parameter('center_tolerance').value
        
        # Calculează deviația orizontală (centru corp vs centru cadru)
        x_deviation = pose.center_x - 0.5  # Negativ = stânga, Pozitiv = dreapta
        
        # Control ANGULAR - aliniază robotul cu persoana - FIX: fără minus
        if abs(x_deviation) > center_tol:
            twist.angular.z = x_deviation * self.max_angular_speed * 2.0
            twist.angular.z = max(min(twist.angular.z, self.max_angular_speed), -self.max_angular_speed)
        else:
            twist.angular.z = 0.0
        
        # Control LINEAR - menține distanța
        height_error = pose.body_height - self.target_body_height
        
        if abs(height_error) > self.height_tolerance:
            # Persoană prea aproape (body_height mare) → mergi înapoi
            # Persoană prea departe (body_height mic) → mergi înainte
            twist.linear.x = -height_error * self.max_linear_speed * 2.0
            twist.linear.x = max(min(twist.linear.x, self.max_linear_speed), -self.max_linear_speed)
        else:
            twist.linear.x = 0.0
        
        self.cmd_pub.publish(twist)
        
        self.get_logger().debug(f'Pose: x={pose.center_x:.2f}, height={pose.body_height:.2f} | '
                               f'cmd: lin={twist.linear.x:.2f}, ang={twist.angular.z:.2f}')
    
    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def safety_check(self):
        time_since_last = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_since_last > self.safety_timeout:
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = PoseFollower()
    
    try:
        while rclpy.ok():
            node.process_keyboard()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        print("\n👋 Închidere Pose Follower...")
    finally:
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
