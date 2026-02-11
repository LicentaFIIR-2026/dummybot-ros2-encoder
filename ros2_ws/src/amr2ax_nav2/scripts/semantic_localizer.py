#!/usr/bin/env python3
"""
semantic_localizer.py
---------------------
Nod ROS2 care localizeaza obiectele detectate de MediaPipe in frame-ul /map
prin proiectia bounding box-ului in raze LiDAR.

Subscrie:
    /mediapipe/detections  (vision_msgs/Detection2DArray)
    /scan                  (sensor_msgs/LaserScan)

Publica:
    /semantic_objects      (geometry_msgs/PoseArray)  - pozitii in /map

Hardware specific:
    Camera: fx=687.5, cx=308.1, montata la x=0.225m z=0.140m
            rotatie: -90 roll -90 yaw fata de base_link
    LiDAR:  LD19, angle_min=0, angle_max=2pi, frame=base_laser
            montat la x=0.185m z=0.210m, rotatie zero
"""

import math
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PointStamped

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ---------------------------------------------------------------------------
# Parametri camera (din /camera/camera_info)
# ---------------------------------------------------------------------------
FX = 687.53846
CX = 308.10513

# ---------------------------------------------------------------------------
# Parametri LiDAR
# Conventie: 0 rad = inainte, creste CCW (standard ROS)
# Camera bearing pozitiv = dreapta in imagine
# LiDAR unghi pozitiv = stanga (CCW)
# Deci: lidar_angle = -bearing_camera
# ---------------------------------------------------------------------------
ANGLE_WINDOW_RAD = math.radians(4.0)   # fereastra cautare raze
RANGE_MIN        = 0.10
RANGE_MAX        = 10.0
FALLBACK_RANGE   = 1.5

# Frame-uri
FRAME_LASER  = 'base_laser'
FRAME_MAP    = 'map'

# Confidence minima
MIN_SCORE = 0.45

# Sync slop intre detectii si scan
SYNC_SLOP = 0.15


class SemanticLocalizer(Node):

    def __init__(self):
        super().__init__('semantic_localizer')

        # Parametri
        self.declare_parameter('min_score',          MIN_SCORE)
        self.declare_parameter('angle_window_deg',   math.degrees(ANGLE_WINDOW_RAD))
        self.declare_parameter('fallback_range',     FALLBACK_RANGE)
        self.declare_parameter('sync_slop',          SYNC_SLOP)

        self.min_score       = self.get_parameter('min_score').value
        angle_win_deg        = self.get_parameter('angle_window_deg').value
        self.angle_window    = math.radians(angle_win_deg)
        self.fallback_range  = self.get_parameter('fallback_range').value
        sync_slop            = self.get_parameter('sync_slop').value

        # TF2
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriberi sincronizati
        self.sub_det  = Subscriber(self, Detection2DArray,
                                   '/mediapipe/detections', qos_profile=qos)
        self.sub_scan = Subscriber(self, LaserScan,
                                   '/scan', qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_det, self.sub_scan],
            queue_size=10,
            slop=sync_slop
        )
        self.sync.registerCallback(self.callback)

        # Publishers
        from rclpy.qos import DurabilityPolicy
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.pub_poses = self.create_publisher(
            PoseArray, '/semantic_objects', qos_pub
        )
        self.pub_debug = self.create_publisher(
            PoseStamped, '/semantic_objects/debug', qos_pub
        )

        # Statistici
        self._n_callbacks     = 0
        self._n_localized     = 0
        self._n_fallback      = 0
        self._n_tf_fail       = 0

        self.get_logger().info(
            f'SemanticLocalizer pornit.\n'
            f'  Fereastra unghi: +/- {angle_win_deg:.1f} grade\n'
            f'  Min score: {self.min_score}\n'
            f'  Sync slop: {sync_slop}s'
        )

    # ------------------------------------------------------------------
    # CALLBACK PRINCIPAL
    # ------------------------------------------------------------------

    def callback(self, det_msg: Detection2DArray, scan_msg: LaserScan):
        self._n_callbacks += 1

        if not det_msg.detections:
            return

        # Pre-proceseaza scan o singura data
        ranges = self._preprocess_scan(scan_msg)
        stamp  = scan_msg.header.stamp

        poses  = []
        labels = []

        for det in det_msg.detections:
            if not det.results:
                continue

            score = det.results[0].hypothesis.score
            label = det.results[0].hypothesis.class_id

            if score < self.min_score:
                continue

            cx_px = det.bbox.center.position.x

            # --- Pas 1: bearing unghi din centrul bbox ---
            bearing_cam = self._pixel_to_bearing(cx_px)

            # --- Pas 2: unghi in frame LiDAR ---
            # Camera dreapta pozitiv, LiDAR CCW pozitiv
            # Deci negam bearing-ul camerei
            lidar_angle = -bearing_cam

            # Normalizam in [0, 2pi] conform conventiei LD19
            lidar_angle = lidar_angle % (2.0 * math.pi)

            # --- Pas 3: raza LiDAR ---
            range_m, fallback = self._find_range(
                lidar_angle, ranges, scan_msg
            )
            if fallback:
                self._n_fallback += 1

            # --- Pas 4: pozitie 3D in frame base_laser ---
            # LiDAR 2D: x=inainte, y=stanga, z=0
            x_laser = range_m * math.cos(lidar_angle)
            y_laser = range_m * math.sin(lidar_angle)

            point = PointStamped()
            point.header.stamp    = stamp
            point.header.frame_id = FRAME_LASER
            point.point.x         = x_laser
            point.point.y         = y_laser
            point.point.z         = 0.0

            # --- Pas 5: transformare in /map ---
            point_map = self._to_map(point)
            if point_map is None:
                self._n_tf_fail += 1
                continue

            self._n_localized += 1

            pose = Pose()
            pose.position.x    = point_map.point.x
            pose.position.y    = point_map.point.y
            pose.position.z    = 0.0
            pose.orientation.w = 1.0

            poses.append(pose)
            labels.append(label)

            self.get_logger().info(
                f'[{label}] score={score:.2f} '
                f'bearing={math.degrees(bearing_cam):.1f}° '
                f'range={range_m:.2f}m '
                f'map=({point_map.point.x:.2f}, {point_map.point.y:.2f})'
                + (' [FALLBACK]' if fallback else '')
            )

        if poses:
            arr = PoseArray()
            arr.header.stamp    = stamp
            arr.header.frame_id = FRAME_MAP
            arr.poses           = poses
            self.pub_poses.publish(arr)

            # Debug: publica primul obiect ca PoseStamped
            ps = PoseStamped()
            ps.header.stamp    = stamp
            ps.header.frame_id = FRAME_MAP
            ps.pose            = poses[0]
            self.pub_debug.publish(ps)

        if self._n_callbacks % 50 == 0:
            self.get_logger().info(
                f'Stats: callbacks={self._n_callbacks} '
                f'localizate={self._n_localized} '
                f'fallback={self._n_fallback} '
                f'tf_fail={self._n_tf_fail}'
            )

    # ------------------------------------------------------------------
    # FUNCTII COMPONENTE
    # ------------------------------------------------------------------

    def _pixel_to_bearing(self, cx_px: float) -> float:
        """
        Calculeaza bearing angle orizontal din coordonata X in pixeli.
        bearing > 0 = obiect la dreapta camerei
        bearing < 0 = obiect la stanga camerei
        """
        return math.atan2((cx_px - CX) / FX, 1.0)

    def _preprocess_scan(self, scan_msg: LaserScan) -> np.ndarray:
        """Converteste LaserScan in numpy array, invalideaza out-of-range."""
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        invalid = (
            ~np.isfinite(ranges) |
            (ranges < scan_msg.range_min) |
            (ranges > scan_msg.range_max)
        )
        ranges[invalid] = 0.0
        return ranges

    def _find_range(self,
                    lidar_angle: float,
                    ranges: np.ndarray,
                    scan_msg: LaserScan) -> tuple[float, bool]:
        """
        Gaseste distanta LiDAR pentru un unghi dat.
        Cauta in fereastra +/- angle_window in jurul unghiului tinta.
        Returneaza (range_m, used_fallback).
        """
        angle_min = scan_msg.angle_min   # 0.0
        angle_inc = scan_msg.angle_increment  # ~0.01254 rad

        idx_center = int((lidar_angle - angle_min) / angle_inc)
        idx_center = max(0, min(idx_center, len(ranges) - 1))

        half_win = max(1, int(self.angle_window / angle_inc))
        idx_start = max(0, idx_center - half_win)
        idx_end   = min(len(ranges), idx_center + half_win + 1)

        window = ranges[idx_start:idx_end]
        valid  = window[(window > RANGE_MIN) & (window < RANGE_MAX)]

        if len(valid) == 0:
            return self.fallback_range, True

        return float(np.min(valid)), False

    def _to_map(self, point: PointStamped):
        """Transforma un punct din base_laser in /map via TF."""
        try:
            return self.tf_buffer.transform(
                point,
                FRAME_MAP,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().debug(f'TF eroare: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
