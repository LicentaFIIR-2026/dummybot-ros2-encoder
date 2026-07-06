#!/usr/bin/env python3
"""
yolo_standalone.py
------------------
Detectie obiecte cu YOLOv8-nano (ULTRA LIGHT)
Înlocuiește mediapipe_standalone.py cu model 3-4x mai rapid
"""

import time
import json
import socket
import os
import threading

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

SOCKET_PATH = '/tmp/mediapipe_detections.sock'  # Keep same path for compatibility

# YOLO OPTIMIZED SETTINGS
DETECTION_HZ = 2.0  # 2 Hz - suficient pentru tracking
CONFIDENCE_THRESHOLD = 0.50
IOU_THRESHOLD = 0.45

# Input resolution
INPUT_WIDTH = 320
INPUT_HEIGHT = 240

# COCO classes (bottle = 39)
TARGET_CLASSES = [39]  # bottle only
CLASS_NAMES = {39: 'bottle'}


class CameraReader(Node):
    """Minimal ROS2 node - buffers latest frame"""
    
    def __init__(self):
        super().__init__('yolo_camera_reader')
        self.bridge = CvBridge()
        self._lock = threading.Lock()
        self._frame = None
        self.create_subscription(Image, '/camera/image_raw', self._cb, 1)
    
    def _cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Resize immediately to save memory
            frame = cv2.resize(frame, (INPUT_WIDTH, INPUT_HEIGHT), interpolation=cv2.INTER_LINEAR)
            with self._lock:
                self._frame = frame
        except Exception as e:
            pass
    
    def get_frame(self):
        with self._lock:
            return self._frame.copy() if self._frame is not None else None


def init_yolo():
    """Initialize YOLOv8-nano detector"""
    print('[yolo] Loading YOLOv8-nano model...')
    
    # Download model if not exists
    model = YOLO('yolov8n.pt')
    
    # Warm-up inference
    dummy = np.zeros((INPUT_HEIGHT, INPUT_WIDTH, 3), dtype=np.uint8)
    model(dummy, verbose=False)
    
    print('[yolo] Model loaded and warmed up')
    return model


def init_socket():
    """Initialize Unix socket for bridge communication"""
    if os.path.exists(SOCKET_PATH):
        os.remove(SOCKET_PATH)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)
    server.settimeout(5.0)
    print(f'[yolo] Socket created: {SOCKET_PATH}')
    print(f'[yolo] Waiting for ros2_bridge...')
    return server


def main():
    print('[yolo] ========================================')
    print('[yolo] YOLOv8-nano Standalone Detector')
    print(f'[yolo] Resolution: {INPUT_WIDTH}x{INPUT_HEIGHT}')
    print(f'[yolo] Detection rate: {DETECTION_HZ} Hz')
    print(f'[yolo] Target: bottle (COCO class 39)')
    print('[yolo] ========================================')
    
    # Init ROS2 + camera
    rclpy.init()
    cam = CameraReader()
    
    # Spin ROS2 on separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(cam,), daemon=True)
    spin_thread.start()
    print('[yolo] Camera reader active')
    
    # Init YOLO
    model = init_yolo()
    
    # Init socket
    server = init_socket()
    conn = None
    interval = 1.0 / DETECTION_HZ
    
    frame_count = 0
    total_inference_time = 0
    
    try:
        while rclpy.ok():
            # Accept connection
            if conn is None:
                try:
                    conn, _ = server.accept()
                    print('[yolo] ros2_bridge connected')
                except socket.timeout:
                    continue
            
            t_start = time.monotonic()
            
            # Get frame
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            
            # YOLOv8 inference
            t_inf = time.monotonic()
            
            results = model(
                frame,
                conf=CONFIDENCE_THRESHOLD,
                iou=IOU_THRESHOLD,
                classes=TARGET_CLASSES,
                verbose=False,
                device='cpu'
            )
            
            inf_ms = (time.monotonic() - t_inf) * 1000.0
            
            # Parse results
            detections = []
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls = int(box.cls[0].cpu().numpy())
                    
                    # Calculate center and dimensions
                    cx = float((x1 + x2) / 2.0)
                    cy = float((y1 + y2) / 2.0)
                    width = float(x2 - x1)
                    height = float(y2 - y1)
                    
                    detections.append({
                        'label': CLASS_NAMES.get(cls, 'unknown'),
                        'score': conf,
                        'cx': cx,
                        'cy': cy,
                        'width': width,
                        'height': height,
                    })
            
            # Send to bridge (same format as MediaPipe for compatibility)
            payload = json.dumps({
                'timestamp': time.time(),
                'detections': detections,
                'inference_ms': inf_ms,
            }) + '\n'
            
            try:
                conn.sendall(payload.encode())
            except (BrokenPipeError, ConnectionResetError):
                print('[yolo] Bridge disconnected, waiting for reconnect...')
                conn.close()
                conn = None
                continue
            
            # Statistics
            frame_count += 1
            total_inference_time += inf_ms
            
            # Log periodically
            if len(detections) > 0:
                print(f'[yolo] inf={inf_ms:.1f}ms detections={len(detections)} {[d["label"] for d in detections]}')
            elif frame_count % 20 == 0:
                avg_inf = total_inference_time / frame_count
                print(f'[yolo] Avg inference: {avg_inf:.1f}ms/frame ({frame_count} frames)')
            
            # Sleep to maintain rate
            elapsed = time.monotonic() - t_start
            sleep_time = max(0.05, interval - elapsed)
            time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print('[yolo] Shutting down...')
    finally:
        if conn:
            conn.close()
        server.close()
        if os.path.exists(SOCKET_PATH):
            os.remove(SOCKET_PATH)
        cam.destroy_node()
        rclpy.shutdown()
        
        # Final stats
        if frame_count > 0:
            avg_inf = total_inference_time / frame_count
            print(f'[yolo] Final stats: {frame_count} frames, avg {avg_inf:.1f}ms/frame')


if __name__ == '__main__':
    main()