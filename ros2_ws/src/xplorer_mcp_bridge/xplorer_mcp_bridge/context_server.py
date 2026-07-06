import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped

import threading
import base64
import json
import math
import os

import uvicorn
from fastapi import FastAPI
from fastapi.responses import JSONResponse

app = FastAPI()
_node_instance = None


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@app.get('/pose')
def get_pose():
    node = _node_instance
    if node is None or node.latest_pose is None:
        return JSONResponse({'error': 'no pose available'}, status_code=503)
    p = node.latest_pose
    return JSONResponse({
        'x': p['x'],
        'y': p['y'],
        'yaw': p['yaw']
    })


@app.get('/camera')
def get_camera():
    node = _node_instance
    if node is None or node.latest_image is None:
        return JSONResponse({'error': 'no image available'}, status_code=503)
    return JSONResponse({
        'format': 'jpeg',
        'data': node.latest_image
    })


@app.get('/graph')
def get_graph():
    node = _node_instance
    if node is None:
        return JSONResponse({'error': 'node not ready'}, status_code=503)
    path = node.get_parameter('route_graph_path').get_parameter_value().string_value
    if not path or not os.path.exists(path):
        return JSONResponse({'error': f'route_graph_path not set or not found: {path}'}, status_code=404)
    with open(path, 'r') as f:
        return JSONResponse(json.load(f))


@app.get('/objects')
def get_objects():
    node = _node_instance
    if node is None:
        return JSONResponse({'error': 'node not ready'}, status_code=503)
    path = node.get_parameter('semantic_objects_path').get_parameter_value().string_value
    if not path or not os.path.exists(path):
        return JSONResponse({'error': f'semantic_objects_path not set or not found: {path}'}, status_code=404)
    with open(path, 'r') as f:
        return JSONResponse(json.load(f))


@app.get('/context')
def get_context():
    pose_resp = get_pose()
    graph_resp = get_graph()
    objects_resp = get_objects()

    pose_data = json.loads(pose_resp.body)
    graph_data = json.loads(graph_resp.body)
    objects_data = json.loads(objects_resp.body)

    return JSONResponse({
        'pose': pose_data,
        'graph': graph_data,
        'objects': objects_data
    })


class ContextServerNode(Node):
    def __init__(self):
        super().__init__('xplorer_context_server')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('semantic_objects_path', '')
        self.declare_parameter('route_graph_path', '')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('pose_topic', '/amcl_pose')

        self.latest_pose = None
        self.latest_image = None

        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        http_port = self.get_parameter('http_port').get_parameter_value().integer_value

        self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._pose_callback,
            10
        )

        self.create_subscription(
            CompressedImage,
            camera_topic,
            self._image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to pose: {pose_topic}')
        self.get_logger().info(f'Subscribed to camera: {camera_topic}')
        self.get_logger().info(f'Starting HTTP server on port {http_port}')

        server_thread = threading.Thread(
            target=lambda: uvicorn.run(app, host='0.0.0.0', port=http_port, log_level='warning'),
            daemon=True
        )
        server_thread.start()

    def _pose_callback(self, msg):
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw
        }

    def _image_callback(self, msg):
        self.latest_image = base64.b64encode(bytes(msg.data)).decode('utf-8')


def main(args=None):
    global _node_instance
    rclpy.init(args=args)
    node = ContextServerNode()
    _node_instance = node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()