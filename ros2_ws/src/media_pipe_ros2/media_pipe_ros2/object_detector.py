#!/usr/bin/env python3
"""
object_detector.py
------------------
Nod MediaPipe Object Detection pentru DummyBot.
Subscrie la /camera/image_raw (nu deschide camera direct).
Publica pe vision_msgs/Detection2DArray - compatibil cu semantic_localizer.py

Inferenta ruleaza pe thread separat ca sa nu blocheze ROS2 executor.

Model: efficientdet_lite0_int8 (~75ms pe Pi 5)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

TARGET_CLASSES = {
    'person', 'chair', 'bottle', 'cup',
    'backpack', 'suitcase', 'box', 'laptop'
}

DEFAULT_MODEL = '/home/saim/mediapipe_models/efficientdet_lite0_int8.tflite'


class ObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('mediapipe_object_detector')

        # Parametri
        self.declare_parameter('model_path',        DEFAULT_MODEL)
        self.declare_parameter('max_results',        5)
        self.declare_parameter('score_threshold',    0.45)
        self.declare_parameter('detection_rate_hz',  2.0)
        self.declare_parameter('publish_debug_image', True)

        model_path       = self.get_parameter('model_path').value
        max_results      = self.get_parameter('max_results').value
        score_threshold  = self.get_parameter('score_threshold').value
        detection_rate   = self.get_parameter('detection_rate_hz').value
        self.pub_debug   = self.get_parameter('publish_debug_image').value

        self.bridge = CvBridge()
        self.detection_interval = 1.0 / detection_rate

        # Buffer frame - accesat din doua threaduri, protejat cu lock
        self._frame_lock   = threading.Lock()
        self._latest_frame = None
        self._latest_stamp = None

        # Statistici
        self._frame_count = 0

        # Initializam detectorul
        self.detector = self._init_detector(model_path, max_results, score_threshold)
        if self.detector is None:
            return

        # Subscriber imagine - callback usor, doar buffereaza
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10
        )

        # Publishers
        self.pub_detections = self.create_publisher(
            Detection2DArray,
            '/mediapipe/detections',
            10
        )
        if self.pub_debug:
            self.pub_image = self.create_publisher(
                Image,
                '/mediapipe/object_detection/image',
                10
            )

        # Thread dedicat pentru inferenta - nu blocheaza ROS2 executor
        self._running = True
        self._inference_thread = threading.Thread(
            target=self._inference_loop,
            name='mediapipe_inference',
            daemon=True
        )
        self._inference_thread.start()

        self.get_logger().info(
            f'MediaPipe Object Detector pornit.\n'
            f'  Model: {model_path}\n'
            f'  Rata procesare: {detection_rate} Hz\n'
            f'  Score threshold: {score_threshold}\n'
            f'  Clase tinta: {TARGET_CLASSES}\n'
            f'  Inferenta pe thread separat: DA'
        )

    # ------------------------------------------------------------------
    # Initializare detector
    # ------------------------------------------------------------------

    def _init_detector(self, model_path, max_results, score_threshold):
        import os
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Model negasit: {model_path}\n'
                f'Descarca cu:\n'
                f'  wget -O {model_path} \\\n'
                f'  https://storage.googleapis.com/mediapipe-models/'
                f'object_detector/efficientdet_lite0/int8/1/efficientdet_lite0.tflite'
            )
            return None

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.ObjectDetectorOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.IMAGE,
            max_results=max_results,
            score_threshold=score_threshold
        )
        try:
            detector = mp_vision.ObjectDetector.create_from_options(options)
            self.get_logger().info('MediaPipe ObjectDetector initializat cu succes.')
            return detector
        except Exception as e:
            self.get_logger().error(f'Eroare initializare detector: {e}')
            return None

    # ------------------------------------------------------------------
    # Callback subscriber - doar buffereaza, nu proceseaza
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image):
        """
        Callback usor - copiaza referinta la mesaj si iese imediat.
        Nu face nicio procesare aici ca sa nu blocheze executorul ROS2.
        """
        with self._frame_lock:
            self._latest_frame = msg
            self._latest_stamp = msg.header.stamp

    # ------------------------------------------------------------------
    # Thread inferenta - complet independent de ROS2 executor
    # ------------------------------------------------------------------

    def _inference_loop(self):
        while self._running and rclpy.ok():
            t_start = time.monotonic()

            with self._frame_lock:
                frame_msg   = self._latest_frame
                frame_stamp = self._latest_stamp

            if frame_msg is not None:
                self._process_frame(frame_msg, frame_stamp)

            elapsed    = time.monotonic() - t_start
            sleep_time = max(0.2, self.detection_interval - elapsed)
            time.sleep(sleep_time)

    def _process_frame(self, frame_msg: Image, stamp):
        """Proceseaza un frame - apelat din thread-ul de inferenta."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                frame_msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().warn(f'CvBridge eroare: {e}')
            return

        # MediaPipe lucreaza cu RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        mp_image  = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Inferenta
        t0 = time.monotonic()
        try:
            results = self.detector.detect(mp_image)
        except Exception as e:
            self.get_logger().warn(f'Eroare inferenta: {e}')
            return
        inference_ms = (time.monotonic() - t0) * 1000.0

        self._frame_count += 1
        if self._frame_count % 30 == 0:
            self.get_logger().info(f'Inference: {inference_ms:.1f}ms')

        # Construim Detection2DArray
        det_array             = Detection2DArray()
        det_array.header.stamp    = stamp
        det_array.header.frame_id = 'camera_link_optical'

        for detection in results.detections:
            if not detection.categories:
                continue

            category = detection.categories[0]
            label    = category.category_name.lower()

            if label not in TARGET_CLASSES:
                continue

            bbox = detection.bounding_box
            cx   = float(bbox.origin_x + bbox.width  / 2.0)
            cy   = float(bbox.origin_y + bbox.height / 2.0)

            det2d = Detection2D()
            det2d.header.stamp    = stamp
            det2d.header.frame_id = 'camera_link_optical'

            det2d.bbox.center.position.x = cx
            det2d.bbox.center.position.y = cy
            det2d.bbox.size_x            = float(bbox.width)
            det2d.bbox.size_y            = float(bbox.height)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score    = float(category.score)
            det2d.results.append(hyp)
            det2d.id = label

            det_array.detections.append(det2d)

        # Publicare thread-safe - publisherii ROS2 sunt thread-safe in rclpy
        self.pub_detections.publish(det_array)

        if self.pub_debug and det_array.detections:
            debug_img = self._draw_detections(cv_image, results)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header.stamp = stamp
            self.pub_image.publish(debug_msg)

    # ------------------------------------------------------------------
    # Debug vizualizare
    # ------------------------------------------------------------------

    def _draw_detections(self, image, results):
        img = image.copy()
        for detection in results.detections:
            if not detection.categories:
                continue
            label = detection.categories[0].category_name.lower()
            if label not in TARGET_CLASSES:
                continue
            score = detection.categories[0].score
            bb    = detection.bounding_box
            cv2.rectangle(
                img,
                (bb.origin_x, bb.origin_y),
                (bb.origin_x + bb.width, bb.origin_y + bb.height),
                (0, 255, 0), 2
            )
            cv2.putText(
                img,
                f'{label} {score:.2f}',
                (bb.origin_x, bb.origin_y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
        return img

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        if self._inference_thread.is_alive():
            self._inference_thread.join(timeout=2.0)
        if self.detector:
            self.detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    # MultiThreadedExecutor ca sa nu blocheze callback-urile
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