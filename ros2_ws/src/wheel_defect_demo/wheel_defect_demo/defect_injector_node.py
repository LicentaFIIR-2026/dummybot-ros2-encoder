#!/usr/bin/env python3
"""
defect_injector_node.py

Comanda robotul sa mearga drept 2m.
Defectul se injecteaza manual (blocare fizica roata).
slip_compensator_node detecteaza din /joint_states reale si opreste robotul.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String


WHEEL_RADIUS     = 0.0345
LINEAR_SPEED     = 0.20
DISTANCE_TARGET  = 2.0

JOINT_NAMES = {
    'front_left_wheel_joint':  'FL',
    'front_right_wheel_joint': 'FR',
    'rear_left_wheel_joint':   'RL',
    'rear_right_wheel_joint':  'RR',
}


class DefectInjectorNode(Node):
    def __init__(self):
        super().__init__('defect_injector_node')

        self.state          = 'IDLE'
        self.start_time     = None
        self.distance_done  = 0.0
        self.last_odom_time = None

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.defect_sub = self.create_subscription(
            String, '/wheel_defect_status', self.defect_cb, 10)

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, '/wheel_defect_status', 10)

        self.create_timer(0.05, self.control_loop)
        self.init_time = self.get_clock().now()

        self.get_logger().info('DefectInjector pornit. Incepe in 2 secunde...')
        self.get_logger().info('Blocheaza manual o roata pentru a testa compensarea.')

    def defect_cb(self, msg: String):
        if msg.data.startswith('DEFECT_ACTIVE') and self.state == 'MOVING':
            self.state = 'DONE'
            self.get_logger().warn(
                f'Defect detectat! Oprire robot. ({msg.data})')

    def joint_cb(self, msg: JointState):
        velocities = {}
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES and i < len(msg.velocity):
                velocities[JOINT_NAMES[name]] = msg.velocity[i]

        if self.state == 'MOVING' and self.last_odom_time is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            dt  = now - self.last_odom_time
            v_fr = velocities.get('FR', 0.0)
            v_rr = velocities.get('RR', 0.0)
            v_avg = (abs(v_fr) + abs(v_rr)) / 2.0 * WHEEL_RADIUS
            self.distance_done += v_avg * dt

        self.last_odom_time = self.get_clock().now().nanoseconds * 1e-9

    def control_loop(self):
        now_sec = (self.get_clock().now().nanoseconds -
                   self.init_time.nanoseconds) * 1e-9

        if self.state == 'IDLE':
            if now_sec >= 2.0:
                self.state = 'MOVING'
                self.start_time = self.get_clock().now().nanoseconds * 1e-9
                self.last_odom_time = self.start_time
                self.get_logger().info('START: Robot merge drept 2m...')
                self.status_pub.publish(String(data='MOVING'))
            return

        if self.state == 'DONE':
            self._send_stop()
            return

        if self.distance_done >= DISTANCE_TARGET:
            self.state = 'DONE'
            self.get_logger().info(
                f'DONE: {self.distance_done:.2f}m parcursi. Robot oprit.')
            self.status_pub.publish(String(data='DONE'))
            self._send_stop()
            return

        self._send_cmd()

        self.get_logger().info(
            f'[MOVING] dist={self.distance_done:.2f}m / {DISTANCE_TARGET}m',
            throttle_duration_sec=0.5)

    def _send_cmd(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = LINEAR_SPEED
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _send_stop(self):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DefectInjectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()