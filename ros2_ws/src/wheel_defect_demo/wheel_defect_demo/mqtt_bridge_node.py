#!/usr/bin/env python3
"""
mqtt_bridge_node.py

Publica datele robotului pe MQTT pentru vizualizare in timp real:
  - Vitezele celor 4 roti (/joint_states)
  - Starea defectului (/wheel_defect_status)
  - cmd_vel original si compensat
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import time


MQTT_BROKER   = "localhost"
MQTT_PORT     = 1883
MQTT_TOPIC_WHEELS  = "aionis/wheel_velocities"
MQTT_TOPIC_DEFECT  = "aionis/defect_status"
MQTT_TOPIC_CMDVEL  = "aionis/cmd_vel"

JOINT_NAMES = {
    'front_left_wheel_joint':  'FL',
    'front_right_wheel_joint': 'FR',
    'rear_left_wheel_joint':   'RL',
    'rear_right_wheel_joint':  'RR',
}


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # MQTT client
        self.mqtt_client = mqtt.Client(client_id="aionis_bridge")
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        self.mqtt_client.loop_start()

        self.last_cmd_vel    = {'linear': 0.0, 'angular': 0.0}
        self.last_cmd_comp   = {'linear': 0.0, 'angular': 0.0}
        self.defect_status   = 'IDLE'

        # Subscribers
        self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(
            TwistStamped, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(
            String, '/wheel_defect_status', self.defect_cb, 10)

        self.get_logger().info('MqttBridge pornit. Publica pe MQTT...')

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'Conectat la MQTT broker pe {MQTT_BROKER}:{MQTT_PORT}')
        else:
            self.get_logger().error(f'Eroare conectare MQTT: {rc}')

    def joint_cb(self, msg: JointState):
        velocities = {}
        for i, name in enumerate(msg.name):
            if name in JOINT_NAMES and i < len(msg.velocity):
                velocities[JOINT_NAMES[name]] = round(msg.velocity[i], 4)

        payload = {
            'timestamp': time.time(),
            'velocities': velocities,
            'defect_status': self.defect_status,
        }
        self.mqtt_client.publish(
            MQTT_TOPIC_WHEELS, json.dumps(payload), qos=0)

    def cmd_vel_cb(self, msg: TwistStamped):
        self.last_cmd_vel = {
            'linear':  round(msg.twist.linear.x, 4),
            'angular': round(msg.twist.angular.z, 4),
        }
        payload = {
            'timestamp': time.time(),
            'original': self.last_cmd_vel,
            'defect_status': self.defect_status,
        }
        self.mqtt_client.publish(
            MQTT_TOPIC_CMDVEL, json.dumps(payload), qos=0)

    def defect_cb(self, msg: String):
        self.defect_status = msg.data
        payload = {
            'timestamp': time.time(),
            'status': msg.data,
        }
        self.mqtt_client.publish(
            MQTT_TOPIC_DEFECT, json.dumps(payload), qos=0)
        self.get_logger().info(f'Defect status: {msg.data}')

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()