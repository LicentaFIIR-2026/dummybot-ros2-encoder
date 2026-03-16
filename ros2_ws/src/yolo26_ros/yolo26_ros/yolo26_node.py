#!/usr/bin/env python3
"""
YOLO26 ROS2 Lifecycle Node
Optimized for Raspberry Pi 5 with Nav2 integration
"""

import rclpy
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge

import time
from typing import Optional

from .yolo26_detector import Yolo26Detector


class Yolo26Node(LifecycleNode):
    """
    YOLO26 Lifecycle Node for ROS2
    
    States:
        - Unconfigured: Initial state
        - Inactive: Model loaded, not processing
        - Active: Processing images
    """
    
    def __init__(self):
        super().__init__('yolo26_detector')
        
        # Will be initialized in on_configure
        self.detector: Optional[Yolo26Detector] = None
        self.bridge = CvBridge()
        self.subscription = None
        self.publisher = None
        self.debug_publisher = None
        
        # Performance tracking
        self.frame_count = 0
        self.total_inference_time = 0.0
        self.last_log_time = time.time()
        
        # Declare parameters
        self.declare_parameter('model_path', '/home/pi/yolo26/yolo26n_ncnn_model')
        self.declare_parameter('input_size', 416)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('num_threads', 4)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detections_topic', '/yolo26/detections')
        self.declare_parameter('debug_image_topic', '/yolo26/debug_image')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('max_detection_rate', 15.0)  # Hz, 0 = unlimited
        self.declare_parameter('class_filter', [])  # Empty = all classes
        
        self.get_logger().info('YOLO26 Node created (Unconfigured)')
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Load model and create publishers"""
        self.get_logger().info('Configuring YOLO26 Node...')
        
        try:
            # Get parameters
            model_path = self.get_parameter('model_path').value
            input_size = self.get_parameter('input_size').value
            conf_thresh = self.get_parameter('confidence_threshold').value
            num_threads = self.get_parameter('num_threads').value
            class_filter = self.get_parameter('class_filter').value
            
            # Convert class_filter to list or None
            if not class_filter:
                class_filter = None
            
            # Initialize detector
            self.detector = Yolo26Detector(
                model_path=model_path,
                input_size=input_size,
                confidence_threshold=conf_thresh,
                num_threads=num_threads,
                class_filter=class_filter
            )
            
            self.get_logger().info(f'Model loaded: {model_path}')
            self.get_logger().info(f'Input size: {input_size}x{input_size}')
            self.get_logger().info(f'Confidence threshold: {conf_thresh}')
            
            # Create publishers
            detections_topic = self.get_parameter('detections_topic').value
            self.publisher = self.create_publisher(
                Detection2DArray,
                detections_topic,
                10
            )
            
            if self.get_parameter('publish_debug_image').value:
                debug_topic = self.get_parameter('debug_image_topic').value
                self.debug_publisher = self.create_publisher(
                    Image,
                    debug_topic,
                    10
                )
            
            self.get_logger().info('Configuration complete')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Start image subscription"""
        self.get_logger().info('Activating YOLO26 Node...')
        
        try:
            # QoS for camera images
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1  # Only latest frame
            )
            
            image_topic = self.get_parameter('image_topic').value
            
            # Callback group for parallel processing
            cb_group = ReentrantCallbackGroup()
            
            self.subscription = self.create_subscription(
                Image,
                image_topic,
                self.image_callback,
                qos,
                callback_group=cb_group
            )
            
            # Reset stats
            self.frame_count = 0
            self.total_inference_time = 0.0
            self.last_log_time = time.time()
            self.last_detection_time = 0.0
            
            self.get_logger().info(f'Subscribed to: {image_topic}')
            self.get_logger().info('YOLO26 Node active')
            
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f'Activation failed: {e}')
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Stop processing"""
        self.get_logger().info('Deactivating YOLO26 Node...')
        
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
        
        # Log final stats
        if self.frame_count > 0:
            avg_time = self.total_inference_time / self.frame_count
            avg_fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info(
                f'Stats: {self.frame_count} frames, '
                f'avg inference: {avg_time*1000:.1f}ms, '
                f'avg FPS: {avg_fps:.1f}'
            )
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Release resources"""
        self.get_logger().info('Cleaning up YOLO26 Node...')
        
        self.detector = None
        
        if self.publisher:
            self.destroy_publisher(self.publisher)
            self.publisher = None
        
        if self.debug_publisher:
            self.destroy_publisher(self.debug_publisher)
            self.debug_publisher = None
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown node"""
        self.get_logger().info('Shutting down YOLO26 Node...')
        return TransitionCallbackReturn.SUCCESS
    
    def image_callback(self, msg: Image):
        """Process incoming image"""
        if self.detector is None:
            return
        
        # Rate limiting
        max_rate = self.get_parameter('max_detection_rate').value
        if max_rate > 0:
            current_time = time.time()
            min_interval = 1.0 / max_rate
            if current_time - self.last_detection_time < min_interval:
                return
            self.last_detection_time = current_time
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run detection
            detections, inference_time = self.detector.detect(cv_image)
            
            # Update stats
            self.frame_count += 1
            self.total_inference_time += inference_time
            
            # Create and publish Detection2DArray
            det_msg = self.create_detection_msg(detections, msg.header)
            self.publisher.publish(det_msg)
            
            # Publish debug image if enabled
            if self.debug_publisher and detections:
                debug_img = self.draw_detections(cv_image, detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, 'bgr8')
                debug_msg.header = msg.header
                self.debug_publisher.publish(debug_msg)
            
            # Periodic logging (every 5 seconds)
            current_time = time.time()
            if current_time - self.last_log_time >= 5.0:
                avg_time = self.total_inference_time / self.frame_count
                fps = self.frame_count / (current_time - self.last_log_time + 0.001)
                self.get_logger().info(
                    f'FPS: {fps:.1f}, Inference: {avg_time*1000:.1f}ms, '
                    f'Detections: {len(detections)}'
                )
                self.frame_count = 0
                self.total_inference_time = 0.0
                self.last_log_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
    
    def create_detection_msg(self, detections, header: Header) -> Detection2DArray:
        """Convert detections to ROS message"""
        msg = Detection2DArray()
        msg.header = header
        
        for det in detections:
            d = Detection2D()
            
            # Bounding box (using pixel coordinates)
            x1, y1, x2, y2 = det.bbox_pixels
            d.bbox.center.position.x = float((x1 + x2) / 2)
            d.bbox.center.position.y = float((y1 + y2) / 2)
            d.bbox.size_x = float(x2 - x1)
            d.bbox.size_y = float(y2 - y1)
            
            # Hypothesis
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.class_id)
            hyp.hypothesis.score = det.confidence
            d.results.append(hyp)
            
            # Add class name as id (for easier debugging)
            d.id = det.class_name
            
            msg.detections.append(d)
        
        return msg
    
    def draw_detections(self, img, detections):
        """Draw bounding boxes on image for debug"""
        import cv2
        
        img_copy = img.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det.bbox_pixels
            
            # Draw box
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f'{det.class_name}: {det.confidence:.2f}'
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img_copy, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
            cv2.putText(img_copy, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        return img_copy


def main(args=None):
    rclpy.init(args=args)
    
    node = Yolo26Node()
    
    # Use MultiThreadedExecutor for better performance
    executor = MultiThreadedExecutor(num_threads=2)
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