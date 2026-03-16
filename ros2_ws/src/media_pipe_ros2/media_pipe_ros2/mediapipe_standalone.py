#!/usr/bin/env python3
"""
mediapipe_standalone.py - ULTRA OPTIMIZED
------------------------------------------
OPTIMIZĂRI DRASTICE:
- 0.5 Hz (o detecție la 2 secunde)
- Rezoluție redusă la 320x240
- Doar 1 obiect detectat
- Doar clasa 'bottle'
- Score threshold ridicat
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

# OPTIMIZĂRI DRASTICE
DETECTION_HZ    = 0.5   # 0.5 Hz = o detecție la 2 secunde!
SCORE_THRESHOLD = 0.50  # Threshold mai strict
MAX_RESULTS     = 1     # Doar 1 obiect

# Rezoluție redusă AGRESIV
CAMERA_WIDTH  = 320  # În loc de 640
CAMERA_HEIGHT = 240  # În loc de 480

# Doar bottle!
TARGET_CLASSES = {'bottle'}


class CameraReader(Node):
    """Nod ROS2 minimal - doar buffereaza ultimul frame."""

    def __init__(self):
        super().__init__('mediapipe_camera_reader')
        self.bridge = CvBridge()
        self._lock  = threading.Lock()
        self._frame = None
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)

    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # RESIZE AGRESIV: 320x240 cu interpolation mai rapid
            frame = cv2.resize(
                frame, 
                (CAMERA_WIDTH, CAMERA_HEIGHT), 
                interpolation=cv2.INTER_NEAREST  # Cel mai rapid
            )
            
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
    print(f'[standalone] Socket: {SOCKET_PATH}')
    print(f'[standalone] Waiting for ros2_bridge...')
    return server


def main():
    print('[standalone] MediaPipe ULTRA OPTIMIZED')
    print(f'[standalone] Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}')
    print(f'[standalone] Detection rate: {DETECTION_HZ} Hz')
    print(f'[standalone] Target classes: {TARGET_CLASSES}')

    # Init ROS2 + camera
    rclpy.init()
    cam = CameraReader()

    # Spin ROS2 pe thread separat
    spin_thread = threading.Thread(target=rclpy.spin, args=(cam,), daemon=True)
    spin_thread.start()
    print('[standalone] Camera reader active')

    # Init detector
    detector = init_detector()
    print('[standalone] Detector ready')

    server = init_socket()
    conn = None
    interval = 1.0 / DETECTION_HZ

    try:
        while rclpy.ok():
            # Accept connection
            if conn is None:
                try:
                    conn, _ = server.accept()
                    print('[standalone] ros2_bridge connected')
                except socket.timeout:
                    continue

            t_start = time.monotonic()

            # Get frame
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue

            # Inference
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

            t_inf = time.monotonic()
            results = detector.detect(mp_image)
            inf_ms = (time.monotonic() - t_inf) * 1000.0

            # Filter + serialize
            detections = []
            for det in results.detections:
                if not det.categories:
                    continue
                cat = det.categories[0]
                label = cat.category_name.lower()
                
                # Doar bottle!
                if label not in TARGET_CLASSES:
                    continue
                
                bb = det.bounding_box
                detections.append({
                    'label': label,
                    'score': float(cat.score),
                    'cx': float(bb.origin_x + bb.width / 2.0),
                    'cy': float(bb.origin_y + bb.height / 2.0),
                    'width': float(bb.width),
                    'height': float(bb.height),
                })

            payload = json.dumps({
                'timestamp': time.time(),
                'detections': detections,
                'inference_ms': inf_ms,
            }) + '\n'

            try:
                conn.sendall(payload.encode())
            except (BrokenPipeError, ConnectionResetError):
                print('[standalone] Bridge disconnected')
                conn.close()
                conn = None
                continue

            # Log doar când detectează ceva sau periodic
            if len(detections) > 0:
                print(f'[standalone] inf={inf_ms:.1f}ms detections={len(detections)} {[d["label"] for d in detections]}')
            elif int(time.monotonic()) % 20 == 0:  # La fiecare 20s
                print(f'[standalone] Active (no detections) inf={inf_ms:.1f}ms')

            elapsed = time.monotonic() - t_start
            sleep_time = max(0.1, interval - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print('[standalone] Shutting down...')
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