#!/usr/bin/env python3
"""
ros2_bridge.py
--------------
Nod ROS2 minimal - doar publica rezultatele de la mediapipe_standalone.
Nu face nicio procesare, nu blocheaza executorul.
Primeste JSON prin Unix socket si publica Detection2DArray.
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

SOCKET_PATH = '/tmp/mediapipe_detections.sock'


class Ros2Bridge(Node):

    def __init__(self):
        super().__init__('mediapipe_ros2_bridge')

        self.pub = self.create_publisher(
            Detection2DArray,
            '/mediapipe/detections',
            10
        )

        # Thread receptor socket - complet separat de executor ROS2
        self._running = True
        self._thread  = threading.Thread(
            target=self._socket_loop,
            name='bridge_socket',
            daemon=True
        )
        self._thread.start()

        self.get_logger().info('MediaPipe ROS2 Bridge pornit.')

    def _socket_loop(self):
        """Conectare la standalone si publicare continua."""
        while self._running and rclpy.ok():
            try:
                self.get_logger().info(
                    f'Conectare la standalone ({SOCKET_PATH})...'
                )
                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.connect(SOCKET_PATH)
                self.get_logger().info('Conectat la mediapipe_standalone.')

                buffer = ''
                while self._running and rclpy.ok():
                    try:
                        data = sock.recv(4096).decode()
                        if not data:
                            break
                        buffer += data

                        # Procesam mesaje complete (delimitare prin \n)
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if line.strip():
                                self._publish_detections(line)

                    except socket.timeout:
                        continue
                    except Exception as e:
                        self.get_logger().warn(f'Eroare socket: {e}')
                        break

                sock.close()

            except (FileNotFoundError, ConnectionRefusedError):
                self.get_logger().warn(
                    'Standalone nu e pornit inca, reîncerc in 2s...'
                )
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().warn(f'Eroare conectare: {e}')
                time.sleep(2.0)

    def _publish_detections(self, json_line: str):
        try:
            data = json.loads(json_line)
        except json.JSONDecodeError:
            return

        det_array             = Detection2DArray()
        det_array.header.stamp    = self.get_clock().now().to_msg()
        det_array.header.frame_id = 'camera_link_optical'

        for d in data.get('detections', []):
            det2d = Detection2D()
            det2d.header.stamp    = det_array.header.stamp
            det2d.header.frame_id = 'camera_link_optical'

            det2d.bbox.center.position.x = d['cx']
            det2d.bbox.center.position.y = d['cy']
            det2d.bbox.size_x            = d['width']
            det2d.bbox.size_y            = d['height']

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d['label']
            hyp.hypothesis.score    = d['score']
            det2d.results.append(hyp)
            det2d.id = d['label']

            det_array.detections.append(det2d)

        self.pub.publish(det_array)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ros2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
