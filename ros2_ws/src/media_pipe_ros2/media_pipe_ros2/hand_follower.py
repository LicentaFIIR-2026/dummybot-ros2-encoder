#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from media_pipe_ros2_msg.msg import MediaPipeHumanHandList
import sys
import select
import termios
import tty

class HandFollower(Node):
    def __init__(self):
        super().__init__('hand_follower')
        
        # Parametri - viteze maxime (editabile în timp real)
        self.max_linear_speed = 18.0   # m/s
        self.max_angular_speed = 17.0   # rad/s
        self.speed_increment = 0.05   # Incrementul pentru ajustare
        
        # Zone de control (normalizat 0-1)
        self.declare_parameter('center_tolerance', 0.15)
        self.declare_parameter('safety_timeout', 1.0)
        
        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.hand_sub = self.create_subscription(
            MediaPipeHumanHandList,
            '/mediapipe/human_hand_list',
            self.hand_callback,
            10
        )
        
        # Safety timeout
        self.last_detection_time = self.get_clock().now()
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.timer = self.create_timer(0.1, self.safety_check)
        
        # Keyboard control
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
    
    def print_instructions(self):
        """Afișează instrucțiunile de control"""
        msg = """
╔══════════════════════════════════════════════════════════╗
║           DummyBot Hand Follower Control                 ║
╚══════════════════════════════════════════════════════════╝

📋 COMENZI TASTATURĂ:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  w/x : Crește/Scade viteza liniară cu 0.05 m/s
  a/d : Crește/Scade viteza angulară cu 0.1 rad/s
  
  s   : STOP forțat (oprește robotul)
  
  q/z : Crește/Scade cu 10% AMBELE viteze
  
  i   : Afișează vitezele curente
  h   : Afișează acest meniu
  
  CTRL+C : Ieșire

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

🤖 CONTROL CU MÂNA:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Mână STÂNGA cadru     → Robot rotește STÂNGA
  Mână DREAPTA cadru    → Robot rotește DREAPTA
  
  Mână CENTRATĂ + SUS   → Robot merge ÎNAINTE
  Mână CENTRATĂ + JOS   → Robot merge ÎNAPOI
  
  Fără mână detectată   → Robot se OPREȘTE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
        print(msg)
        self.print_current_speeds()
    
    def print_current_speeds(self):
        """Afișează vitezele curente"""
        print(f"⚙️  Viteze curente: Linear = {self.max_linear_speed:.2f} m/s | Angular = {self.max_angular_speed:.2f} rad/s")
    
    def get_key(self):
        """Citește tastă apăsată (non-blocking)"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def process_keyboard(self):
        """Procesează input de la tastatură"""
        key = self.get_key()
        
        if key == 'w':
            self.max_linear_speed += self.speed_increment
            print(f"⬆️  Viteza liniară: {self.max_linear_speed:.2f} m/s")
        elif key == 'x':
            self.max_linear_speed = max(0.0, self.max_linear_speed - self.speed_increment)
            print(f"⬇️  Viteza liniară: {self.max_linear_speed:.2f} m/s")
        elif key == 'a':
            self.max_angular_speed += 0.1
            print(f"⬆️  Viteza angulară: {self.max_angular_speed:.2f} rad/s")
        elif key == 'd':
            self.max_angular_speed = max(0.0, self.max_angular_speed - 0.1)
            print(f"⬇️  Viteza angulară: {self.max_angular_speed:.2f} rad/s")
        elif key == 'q':
            self.max_linear_speed *= 1.1
            self.max_angular_speed *= 1.1
            print(f"📈 +10%: Linear = {self.max_linear_speed:.2f} m/s | Angular = {self.max_angular_speed:.2f} rad/s")
        elif key == 'z':
            self.max_linear_speed *= 0.9
            self.max_angular_speed *= 0.9
            print(f"📉 -10%: Linear = {self.max_linear_speed:.2f} m/s | Angular = {self.max_angular_speed:.2f} rad/s")
        elif key == 's':
            self.stop_robot()
            print("🛑 STOP forțat!")
        elif key == 'i':
            self.print_current_speeds()
        elif key == 'h':
            self.print_instructions()
        elif key == '\x03':  # CTRL+C
            raise KeyboardInterrupt
    
    def hand_callback(self, msg):
        """Procesează detectarea mâinii și generează comenzi"""
        
        # Verifică dacă există mână detectată
        if msg.num_humans == 0:
            self.stop_robot()
            return
        
        hand_data = msg.human_hand_list
        
        # Prioritate: mâna dreaptă (pentru control)
        if hand_data.right_hand_key_points[0].x != 0.0:
            hand_points = hand_data.right_hand_key_points
            hand_type = "RIGHT"
        elif hand_data.left_hand_key_points[0].x != 0.0:
            hand_points = hand_data.left_hand_key_points
            hand_type = "LEFT"
        else:
            self.stop_robot()
            return
        
        # Calculează centrul palmei (landmark 9 - middle finger MCP)
        palm_x = hand_points[9].x
        palm_y = hand_points[9].y
        palm_z = hand_points[9].z
        
        # Update timing
        self.last_detection_time = self.get_clock().now()
        
        # Control bazat pe poziție
        self.position_control(palm_x, palm_y, palm_z, hand_type)
    
    def position_control(self, palm_x, palm_y, palm_z, hand_type):
        """Control bazat pe poziția mâinii în cadru"""
        
        twist = Twist()
        
        center_tol = self.get_parameter('center_tolerance').value
        
        # Calculează deviația de la centru (0.5, 0.5)
        x_deviation = palm_x - 0.5  # Negativ = stânga, Pozitiv = dreapta
        y_deviation = 0.5 - palm_y  # Pozitiv = sus, Negativ = jos
        
        # Control angular (rotație) bazat pe poziția orizontală - FIX: fără minus
        if abs(x_deviation) > center_tol:
            # Rotește proporțional cu deviația
            twist.angular.z = x_deviation * self.max_angular_speed * 2.0
            twist.angular.z = max(min(twist.angular.z, self.max_angular_speed), -self.max_angular_speed)
        else:
            twist.angular.z = 0.0
        
        # Control linear bazat pe "distanță" (aproximare cu z și mărimea mâinii)
        distance_factor = -palm_z  # Inversăm pentru logică intuitivă
        
        # Mișcare înainte/înapoi bazat pe poziția verticală
        if abs(x_deviation) < center_tol:
            if y_deviation > 0.1:  # Mână SUS → Înainte
                twist.linear.x = self.max_linear_speed * min(distance_factor * 2.0, 1.0)
            elif y_deviation < -0.1:  # Mână JOS → Înapoi
                twist.linear.x = -self.max_linear_speed * 0.7  # 70% din viteza max pentru înapoi
            else:
                twist.linear.x = 0.0
        else:
            twist.linear.x = 0.0
        
        self.cmd_pub.publish(twist)
    
    def stop_robot(self):
        """Oprește robotul"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
    
    def safety_check(self):
        """Verifică timeout pentru siguranță"""
        time_since_last = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        
        if time_since_last > self.safety_timeout:
            self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    node = HandFollower()
    
    try:
        while rclpy.ok():
            node.process_keyboard()  # Procesează input tastatură
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        print("\n👋 Închidere Hand Follower...")
    finally:
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
