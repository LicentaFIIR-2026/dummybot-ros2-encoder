#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import serial
import time
import math

class ESP32HardwareInterface(Node):
    def __init__(self):
        super().__init__('esp32_hardware_interface')
        
        # Parametri robot
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.0325)  # 65mm diametru / 2
        self.declare_parameter('wheel_separation', 0.3863)  # Lx
        self.declare_parameter('ticks_per_revolution', 1760)
        self.declare_parameter('update_rate', 30.0)  # Hz
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Setup serial
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        
        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2)
            self.get_logger().info(f'Connected to ESP32 on {port}')
            
            # Reset encoders
            self.serial.write(b'r\n')
            time.sleep(0.1)
            
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            return
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_encoder_ticks = [0, 0, 0, 0]  # FL, FR, RL, RR
        
        # Timer for reading encoders and publishing odometry
        rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0/rate, self.update_callback)
        
        self.get_logger().info('ESP32 Hardware Interface started!')
    
    def cmd_vel_callback(self, msg):
        """Convert Twist to motor speeds and send to ESP32 - 4 INDEPENDENT MOTORS"""
        linear = msg.linear.x
        angular = msg.angular.z
        self.get_logger().info(f'cmd_vel: linear={linear:.3f}, angular={angular:.3f}')
        
        wheel_radius = self.get_parameter('wheel_radius').value
        wheel_sep = self.get_parameter('wheel_separation').value
        ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        
        # Differential drive kinematics pentru skid-steer (toate roțile se mișcă la fel pe aceeași parte)
        left_vel = linear - (angular * wheel_sep / 2.0)
        right_vel = linear + (angular * wheel_sep / 2.0)
        
        # Convert m/s to ticks/frame (30Hz = 0.033s per frame)
        meters_per_tick = (2 * math.pi * wheel_radius) / ticks_per_rev
        
        # Calculează ticks pentru fiecare motor
        left_ticks = int(left_vel / meters_per_tick / 30.0)
        right_ticks = int(right_vel / meters_per_tick / 30.0)
        
        # Pentru skid-steer: FL și RL = left, FR și RR = right
        fl_ticks = left_ticks
        fr_ticks = right_ticks
        rl_ticks = left_ticks
        rr_ticks = right_ticks
        
        # Send command to ESP32 - ACUM 4 VALORI
        cmd = f'm {fl_ticks} {fr_ticks} {rl_ticks} {rr_ticks}\n'
        self.get_logger().info(f'Sending: {cmd.strip()}')
        try:
            self.serial.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def update_callback(self):
        """Read encoders and update odometry"""
        #self.get_logger().info('Timer triggered!')
        
        try:
            # Flush orice date vechi
            self.serial.reset_input_buffer()
            
            # Trimite comanda
            self.serial.write(b'e\n')
            self.serial.flush()
            
            # Așteaptă răspuns
            time.sleep(0.1)
            
            #self.get_logger().info(f'Bytes waiting: {self.serial.in_waiting}')  # ADAUGĂ
            
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode().strip()
                #self.get_logger().info(f'Received: [{line}]')
                
                # Ignoră mesajele OK
                if line == "OK":
                    return

                values = line.split()
                if len(values) == 4:
                    fl = int(values[0])
                    fr = int(values[1])
                    rl = int(values[2])
                    rr = int(values[3])
                    
                    encoder_ticks = [fl, fr, rl, rr]
                    #self.get_logger().info(f'PARSED: FL={fl} FR={fr} RL={rl} RR={rr}') 
                    self.compute_odometry(encoder_ticks)
                else:
                    self.get_logger().warn(f'Invalid data: {line}')
            else:
                self.get_logger().warn('No data in buffer!')
                    
        except Exception as e:
            self.get_logger().error(f'Exception: {type(e).__name__}: {e}')  # MAI DETALIAT
    
    def compute_odometry(self, encoder_ticks):
        """Compute odometry from encoder ticks"""
        # Calculate deltas
        delta_ticks = [
            encoder_ticks[i] - self.last_encoder_ticks[i]
            for i in range(4)
        ]
        #self.get_logger().info(f'Delta ticks: FL={delta_ticks[0]} FR={delta_ticks[1]} RL={delta_ticks[2]} RR={delta_ticks[3]}')
        self.last_encoder_ticks = encoder_ticks
        
        # Average left and right
        left_ticks = (delta_ticks[0] + delta_ticks[2]) / 2.0  # FL + RL
        right_ticks = (delta_ticks[1] + delta_ticks[3]) / 2.0  # FR + RR
        
        # Convert ticks to meters
        wheel_radius = self.get_parameter('wheel_radius').value
        ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        meters_per_tick = (2 * math.pi * wheel_radius) / ticks_per_rev
        
        left_dist = left_ticks * meters_per_tick
        right_dist = right_ticks * meters_per_tick

        #self.get_logger().info(f'left_dist={left_dist:.4f}, right_dist={right_dist:.4f}')
        
        # Compute robot displacement
        dist = (left_dist + right_dist) / 2.0
        #self.get_logger().info(f'dist={dist:.4f}')  # ȘI AICI
        wheel_sep = self.get_parameter('wheel_separation').value
        dtheta = (right_dist - left_dist) / wheel_sep
        
        # Update pose
        self.theta += dtheta
        dx = dist * math.cos(self.theta)
        dy = dist * math.sin(self.theta)
        self.x += dx
        self.y += dy
        
        # Compute velocities
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 0:
            vx = dist / dt
            vth = dtheta / dt
        else:
            vx = 0.0
            vth = 0.0
        
        # Publish odometry
        self.publish_odometry(vx, vth)
    
    def publish_odometry(self, vx, vth):
        """Publish odometry message and TF"""
        current_time = self.get_clock().now().to_msg()
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        
        # TF transform
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32HardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
