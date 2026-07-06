#!/usr/bin/env python3
"""
slip_compensator_node.py

Detecteaza defectul pe orice roata din /joint_states.
Cand defectul e confirmat, opreste robotul complet.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from collections import deque


JOINT_NAMES = {
    'front_left_wheel_joint':  'FL',
    'front_right_wheel_joint': 'FR',
    'rear_left_wheel_joint':   'RL',
    'rear_right_wheel_joint':  'RR',
}

DEFECT_THRESHOLD = 0.40   # sub 30% din referinta = defect
CONFIRM_WINDOW   = 3      # sample-uri consecutive pentru confirmare


class SlipCompensatorNode(Node):
    def __init__(self):
        super().__init__('slip_compensator_node')

        self.velocities       = {'FL': 0.0, 'FR': 0.0, 'RL': 0.0, 'RR': 0.0}
        self.defect_history   = deque(maxlen=CONFIRM_WINDOW)
        self.defect_confirmed = False
        self.active           = False
        self.fault_wheel      = None

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.status_sub = self.create_subscription(
            String, '/wheel_defect_status', self.status_cb, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        self.defect_pub = self.create_publisher(
            String, '/wheel_defect_status', 10)

        self.get_logger().info('SlipCompensator pornit. Monitorizeaza /joint_states...')

    def status_cb(self, msg: String):
        if msg.data == 'MOVING':
            self.active = True
            self.defect_confirmed = False
            self.fault_wheel = None
            self.defect_history.clear()
            self.get_logger().info('Compensator activ, monitorizeaza...')
        elif msg.data == 'DONE':
            self.active = False
            self.get_logger().info('Compensator dezactivat.')

    def joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES and i < len(msg.velocity):
                self.velocities[JOINT_NAMES[name]] = abs(msg.velocity[i])

        if self.active and not self.defect_confirmed:
            self._detect()

    def _detect(self):
        speeds = list(self.velocities.values())
        wheels = list(self.velocities.keys())

        # Referinta: media rotilor care se misca
        moving = [s for s in speeds if s > 0.05]
        if len(moving) < 2:
            return

        import statistics
        ref = statistics.median(moving)

        # Verifica fiecare roata
        fault_found = False
        for w, v in self.velocities.items():
            if ref > 0.05 and (v / ref) < DEFECT_THRESHOLD:
                fault_found = True
                self.fault_wheel = w
                break

        self.defect_history.append(fault_found)

        if len(self.defect_history) == CONFIRM_WINDOW and all(self.defect_history):
            self.defect_confirmed = True
            self.get_logger().warn(
                f'[DEFECT CONFIRMAT] Roata {self.fault_wheel} oprita. '
                f'Viteza: {self.velocities[self.fault_wheel]:.3f} rad/s '
                f'(referinta: {ref:.3f} rad/s)')

            # Opreste robotul
            self._send_stop()

            # Publica status pentru dashboard
            self.defect_pub.publish(
                String(data=f'DEFECT_ACTIVE:{self.fault_wheel}'))

    def _send_stop(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().warn('ROBOT OPRIT din cauza defectului.')


def main(args=None):
    rclpy.init(args=args)
    node = SlipCompensatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()