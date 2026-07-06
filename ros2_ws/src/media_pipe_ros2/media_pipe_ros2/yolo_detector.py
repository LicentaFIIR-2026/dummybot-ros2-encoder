#!/usr/bin/env python3
"""
yolo_detector.py
----------------
Nod YOLOv8-nano Object Detection pentru DummyBot (ULTRA OPTIMIZED)
Subscrie la /camera/image_raw
Publica pe vision_msgs/Detection2DArray - compatibil cu semantic_localizer.py
Inferenta ruleaza pe thread separat ca sa nu blocheze ROS2 executor.
Model: YOLOv8n (~15-20ms pe Pi 5) - 4x mai rapid decât MediaPipe!
"""

import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO

# COCO class IDs
TARGET_CLASSES = [39]  # bottle only
CLASS_NAMES = {39: 'bottle'}

# Defaults
DEFAULT_MODEL = 'yolov8n.pt'
INPUT_WIDTH = 128 #320
INPUT_HEIGHT = 96 #240


class YOLODetectorNode(Node):
    
    def __init__(self):
        super().__init__('yolo_object_detector')
        
        # Parameters
        self.declare_parameter('model_path', DEFAULT_MODEL)
        self.declare_parameter('max_results', 1)
        self.declare_parameter('score_threshold', 0.50)
        self.declare_parameter('detection_rate_hz', 2.0)
        self.declare_parameter('input_width', INPUT_WIDTH)
        self.declare_parameter('input_height', INPUT_HEIGHT)
        
        model_path = self.get_parameter('model_path').value
        self.max_results = self.get_parameter('max_results').value
        self.score_threshold = self.get_parameter('score_threshold').value
        detection_rate = self.get_parameter('detection_rate_hz').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        
        self.get_logger().info(f'🚀 YOLO Detector starting...')
        self.get_logger().info(f'  Model: {model_path}')
        self.get_logger().info(f'  Resolution: {self.input_width}x{self.input_height}')
        self.get_logger().info(f'  Rate: {detection_rate} Hz')
        self.get_logger().info(f'  Confidence: {self.score_threshold}')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # YOLO model
        self.get_logger().info('Loading YOLO model...')
        self.model = YOLO(model_path)
        
        # Warm-up
        dummy = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
        self.model(dummy, verbose=False)
        self.get_logger().info('✓ Model loaded and warmed up')
        
        # Subscriber
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1
        )
        
        # Publisher
        self.pub_detections = self.create_publisher(
            Detection2DArray,
            '/mediapipe/detections',  # Same topic for compatibility
            10
        )
        
        # Threading
        self._lock = threading.Lock()
        self._latest_frame = None
        self._processing = False
        
        # Detection thread
        self.detection_interval = 1.0 / detection_rate
        self._running = True
        self._detection_thread = threading.Thread(
            target=self._detection_loop,
            daemon=True
        )
        self._detection_thread.start()
        
        # Stats
        self._n_detections = 0
        self._total_inference_time = 0
        
        self.get_logger().info('✅ YOLO Detector ready')
    
    def image_callback(self, msg: Image):
        """Buffer latest frame (non-blocking)"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Resize immediately
            frame = cv2.resize(
                frame,
                (self.input_width, self.input_height),
                interpolation=cv2.INTER_LINEAR
            )
            
            with self._lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
    
    def _detection_loop(self):
        """Detection thread - runs at fixed rate"""
        while self._running and rclpy.ok():
            t_start = time.monotonic()
            
            # Get latest frame
            with self._lock:
                frame = self._latest_frame
            
            if frame is None:
                time.sleep(0.1)
                continue
            
            # Run detection
            detections = self._detect(frame)
            
            # Publish
            if detections is not None:
                self.pub_detections.publish(detections)
            
            # Maintain rate
            elapsed = time.monotonic() - t_start
            sleep_time = max(0.01, self.detection_interval - elapsed)
            time.sleep(sleep_time)
    
    def _detect(self, frame):
        """Run YOLO inference"""
        t_inf = time.monotonic()
        
        try:
            results = self.model(
                frame,
                conf=self.score_threshold,
                classes=TARGET_CLASSES,
                verbose=False,
                device='cpu',
                max_det=self.max_results,
                imgsz=128,
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return None
        
        inf_ms = (time.monotonic() - t_inf) * 1000.0
        
        # Update stats
        self._n_detections += 1
        self._total_inference_time += inf_ms
        
        # Parse results
        det_array = Detection2DArray()
        det_array.header.stamp = self.get_clock().now().to_msg()
        det_array.header.frame_id = 'camera_link_optical'
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())
                
                # Create Detection2D
                det2d = Detection2D()
                det2d.header.stamp = det_array.header.stamp
                det2d.header.frame_id = 'camera_link_optical'
                
                # Bounding box
                det2d.bbox.center.position.x = float((x1 + x2) / 2.0)
                det2d.bbox.center.position.y = float((y1 + y2) / 2.0)
                det2d.bbox.size_x = float(x2 - x1)
                det2d.bbox.size_y = float(y2 - y1)
                
                # Hypothesis
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = CLASS_NAMES.get(cls, 'unknown')
                hyp.hypothesis.score = conf
                det2d.results.append(hyp)
                det2d.id = CLASS_NAMES.get(cls, 'unknown')
                
                det_array.detections.append(det2d)
        
        # Log periodically
        if len(det_array.detections) > 0:
            labels = [d.id for d in det_array.detections]
            self.get_logger().info(
                f'Detected: {labels} (inf={inf_ms:.1f}ms)',
                throttle_duration_sec=2.0
            )
        elif self._n_detections % 50 == 0:
            avg_inf = self._total_inference_time / self._n_detections
            self.get_logger().info(
                f'Stats: {self._n_detections} frames, avg {avg_inf:.1f}ms/frame',
                throttle_duration_sec=10.0
            )
        
        return det_array
    
    def destroy_node(self):
        """Cleanup"""
        self._running = False
        if self._detection_thread.is_alive():
            self._detection_thread.join(timeout=2.0)
        
        if self._n_detections > 0:
            avg_inf = self._total_inference_time / self._n_detections
            self.get_logger().info(
                f'Final stats: {self._n_detections} detections, '
                f'avg {avg_inf:.1f}ms/frame'
            )
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
