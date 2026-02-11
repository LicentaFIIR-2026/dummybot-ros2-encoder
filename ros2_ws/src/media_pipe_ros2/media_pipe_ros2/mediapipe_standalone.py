#!/usr/bin/env python3
"""
mediapipe_standalone.py
-----------------------
Proces separat de Nav2/ros2_control.
Citeste /camera/image_raw prin rclpy minimal (thread separat).
Face inferenta MediaPipe EfficientDet pe thread principal.
Trimite rezultatele prin Unix socket catre ros2_bridge.py
"""

import time
import json
import socket
import os
import threading

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SOCKET_PATH     = '/tmp/mediapipe_detections.sock'
MODEL_PATH      = '/home/saim/mediapipe_models/efficientdet_lite0_int8.tflite'
DETECTION_HZ    = 3.0
SCORE_THRESHOLD = 0.45
MAX_RESULTS     = 5

TARGET_CLASSES = {
    'person', 'chair', 'bottle', 'cup',
    'backpack', 'suitcase', 'laptop', 'tv',
    'couch', 'dining table', 'potted plant'
}


class CameraReader(Node):
    """Nod ROS2 minimal - doar buffereaza ultimul frame."""

    def __init__(self):
        super().__init__('mediapipe_camera_reader')
        self.bridge = CvBridge()
        self._lock  = threading.Lock()
        self._frame = None
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self._cb,
            1
        )

    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._lock:
                self._frame = frame
        except Exception:
            pass

    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


def init_detector():
    base_options = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
    options = mp_vision.ObjectDetectorOptions(
        base_options=base_options,
        running_mode=mp_vision.RunningMode.IMAGE,
        max_results=MAX_RESULTS,
        score_threshold=SCORE_THRESHOLD
    )
    return mp_vision.ObjectDetector.create_from_options(options)


def init_socket():
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)
    server.settimeout(5.0)
    print(f'[standalone] Socket creat: {SOCKET_PATH}')
    print(f'[standalone] Astept conexiune de la ros2_bridge...')
    return server


def main():
    print('[standalone] Pornire MediaPipe Standalone...')

    # Initializam ROS2 si camera reader
    rclpy.init()
    cam = CameraReader()

    # Spin ROS2 pe thread separat - doar pentru a primi imagini
    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(cam,),
        daemon=True
    )
    spin_thread.start()
    print('[standalone] Camera reader pornit.')

    # Initializam detectorul
    detector = init_detector()
    print('[standalone] Detector initializat.')

    server  = init_socket()
    conn    = None
    interval = 1.0 / DETECTION_HZ

    try:
        while rclpy.ok():
            # Acceptam conexiune de la bridge
            if conn is None:
                try:
                    conn, _ = server.accept()
                    print('[standalone] ros2_bridge conectat.')
                except socket.timeout:
                    continue

            t_start = time.monotonic()

            # Citim ultimul frame disponibil
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.05)
                continue

            # Inferenta
            rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

            t_inf    = time.monotonic()
            results  = detector.detect(mp_image)
            inf_ms   = (time.monotonic() - t_inf) * 1000.0

            # Filtram si serializam
            detections = []
            for det in results.detections:
                if not det.categories:
                    continue
                cat   = det.categories[0]
                label = cat.category_name.lower()
                if label not in TARGET_CLASSES:
                    continue
                bb = det.bounding_box
                detections.append({
                    'label':  label,
                    'score':  float(cat.score),
                    'cx':     float(bb.origin_x + bb.width  / 2.0),
                    'cy':     float(bb.origin_y + bb.height / 2.0),
                    'width':  float(bb.width),
                    'height': float(bb.height),
                })

            payload = json.dumps({
                'timestamp':    time.time(),
                'detections':   detections,
                'inference_ms': inf_ms,
            }) + '\n'

            try:
                conn.sendall(payload.encode())
            except (BrokenPipeError, ConnectionResetError):
                print('[standalone] Bridge deconectat, astept reconectare...')
                conn.close()
                conn = None
                continue

            if len(detections) > 0 or int(time.monotonic()) % 10 == 0:
                print(f'[standalone] inf={inf_ms:.1f}ms '
                      f'detectii={len(detections)} '
                      f'{[d["label"] for d in detections]}')

            elapsed    = time.monotonic() - t_start
            sleep_time = max(0.1, interval - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print('[standalone] Oprire...')
    finally:
        detector.close()
        if conn:
            conn.close()
        server.close()
        if os.path.exists(SOCKET_PATH):
            os.remove(SOCKET_PATH)
        cam.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()