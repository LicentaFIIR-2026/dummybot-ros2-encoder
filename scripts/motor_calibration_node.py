#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from builtin_interfaces.msg import Duration

class MotorCalibrationNode(Node):
    def __init__(self):
        super().__init__('motor_calibration')
        
        # Parametri calibrare
        self.declare_parameter('motor_fl_factor', 1.0)
        self.declare_parameter('motor_fr_factor', 1.0)
        self.declare_parameter('motor_rl_factor', 1.0)
        self.declare_parameter('motor_rr_factor', 1.0)
        
        # Subscribe la comenzi de la diff_drive_controller
        self.sub = self.create_subscription(
            DynamicJointState,
            '/diff_drive_controller/joint_commands',
            self.callback,
            10
        )
        
        # Publish comenzi calibrate către hardware
        self.pub = self.create_publisher(
            DynamicJointState,
            '/diff_drive_controller/joint_commands_calibrated',
            10
        )
        
        self.get_logger().info('Motor Calibration Node started')
        self.get_logger().info(f'Factors: FL={self.get_parameter("motor_fl_factor").value}, '
                              f'FR={self.get_parameter("motor_fr_factor").value}, '
                              f'RL={self.get_parameter("motor_rl_factor").value}, '
                              f'RR={self.get_parameter("motor_rr_factor").value}')
        
    def callback(self, msg):
        factors = [
            self.get_parameter('motor_fl_factor').value,
            self.get_parameter('motor_fr_factor').value,
            self.get_parameter('motor_rl_factor').value,
            self.get_parameter('motor_rr_factor').value
        ]
        
        # Aplică factori la viteze
        calibrated = DynamicJointState()
        calibrated.header = msg.header
        calibrated.joint_names = msg.joint_names
        
        for i, interface in enumerate(msg.interface_values):
            calibrated_interface = interface
            if interface.interface_names and 'velocity' in interface.interface_names[0]:
                # Aplică factorul corespunzător
                joint_idx = self.get_joint_index(msg.joint_names[i])
                if joint_idx is not None:
                    calibrated_interface.values = [interface.values[0] * factors[joint_idx]]
            calibrated.interface_values.append(calibrated_interface)
        
        self.pub.publish(calibrated)
    
    def get_joint_index(self, joint_name):
        if 'front_left' in joint_name:
            return 0
        elif 'front_right' in joint_name:
            return 1
        elif 'rear_left' in joint_name:
            return 2
        elif 'rear_right' in joint_name:
            return 3
        return None

def main():
    rclpy.init()
    node = MotorCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
