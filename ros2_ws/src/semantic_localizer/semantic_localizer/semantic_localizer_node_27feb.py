#!/usr/bin/env python3
"""
Semantic Localizer Node
=======================
Fuses YOLO 2D detections with LiDAR 2D scan to produce object positions in /map frame.

Pipeline:
  1. Receive Detection2DArray from YOLO + LaserScan
  2. Map bounding box horizontal extent → angular range in LiDAR
  3. Extract median distance from relevant LiDAR rays
  4. Transform point from base_laser frame → /map frame via TF2
  5. Publish SemanticDetection markers + custom topic

Designed for Raspberry Pi 5 — minimal CPU overhead (~2-5% at 3Hz).

Author: FIIR / SAIM Xplorer Project
"""

import json
import math
import os
import sys
import copy
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav2_msgs.srv import SetRouteGraph
from std_srvs.srv import Trigger

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — needed for PointStamped transform

from semantic_localizer.semantic_map_manager import SemanticMapManager


# ─── Data structures ───────────────────────────────────────────────

@dataclass
class SemanticObject:
    """A detected object with map-frame position."""
    class_name: str
    class_id: int
    confidence: float
    x_map: float
    y_map: float
    distance: float              # distance from robot in meters
    timestamp: float             # ROS time in seconds
    bbox_center_px: Tuple[float, float] = (0.0, 0.0)
    bbox_size_px: Tuple[float, float] = (0.0, 0.0)
    lidar_rays_used: int = 0     # how many valid LiDAR rays contributed
    angular_width_deg: float = 0.0


@dataclass
class TrackedObject:
    """Object tracked across multiple detections for temporal filtering."""
    class_name: str
    positions: List[Tuple[float, float]] = field(default_factory=list)
    confidences: List[float] = field(default_factory=list)
    last_seen: float = 0.0
    stable_x: float = 0.0
    stable_y: float = 0.0
    detection_count: int = 0


# ─── Color map for visualization ───────────────────────────────────

CLASS_COLORS = {
    'person':     (1.0, 0.2, 0.2, 0.9),  # red
    'chair':      (0.2, 0.6, 1.0, 0.9),  # blue
    'door':       (0.2, 1.0, 0.2, 0.9),  # green
    'bottle':     (1.0, 1.0, 0.2, 0.9),  # yellow
    'cup':        (1.0, 0.5, 0.0, 0.9),  # orange
    'backpack':   (0.8, 0.2, 1.0, 0.9),  # purple
    'suitcase':   (0.0, 1.0, 1.0, 0.9),  # cyan
    'bench':      (0.6, 0.4, 0.2, 0.9),  # brown
    'potted plant': (0.0, 0.8, 0.4, 0.9),
    'default':    (0.8, 0.8, 0.8, 0.9),  # gray
}


class SemanticLocalizerNode(Node):
    """
    Fuses camera-based YOLO detections with 2D LiDAR scan
    to produce map-frame semantic object positions.
    """

    def __init__(self):
        super().__init__('semantic_localizer')

        # ── Declare parameters ──────────────────────────────────
        self.declare_parameter('min_confidence', 0.45)
        self.declare_parameter('max_process_rate_hz', 3.0)
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('detections_topic', '/yolo26/detections')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'base_laser')
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('camera_lidar_yaw_offset', 0.0)

        # Tracking / filtering
        self.declare_parameter('tracking_distance_threshold', 0.5)
        self.declare_parameter('tracking_timeout', 5.0)
        self.declare_parameter('min_detections_stable', 3)
        self.declare_parameter('median_filter_window', 5)

        # LiDAR ray selection
        self.declare_parameter('ray_percentile', 25.0)
        self.declare_parameter('min_valid_rays', 2)
        self.declare_parameter('max_object_distance', 8.0)

        # Semantic Map Manager — persistence & route graph
        self.declare_parameter('semantic_objects_filepath',
                               os.path.expanduser('~/saim_xplorer/maps/semantic_objects.geojson'))
        self.declare_parameter('route_graph_filepath',
                               os.path.expanduser('~/saim_xplorer/maps/route_graph_fiir.geojson'))
        self.declare_parameter('output_graph_filepath',
                               os.path.expanduser('~/saim_xplorer/maps/route_graph_fiir_semantic.geojson'))
        self.declare_parameter('save_interval', 30.0)
        self.declare_parameter('cleanup_interval', 10.0)
        self.declare_parameter('edge_proximity', 1.5)

        # Route Server auto-reload
        self.declare_parameter('nav2_graph_filepath',
                               os.path.expanduser('~/maps/route_graph_nav2.geojson'))
        self.declare_parameter('convert_script_path',
                               os.path.expanduser('~/convert_geojson_for_route_server.py'))

        # ── Read parameters ─────────────────────────────────────
        self.min_confidence = self.get_parameter('min_confidence').value
        self.max_rate = self.get_parameter('max_process_rate_hz').value
        self.min_interval = 1.0 / self.max_rate if self.max_rate > 0 else 0.33
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.cam_lidar_yaw_offset = self.get_parameter('camera_lidar_yaw_offset').value
        self.tracking_dist_thresh = self.get_parameter('tracking_distance_threshold').value
        self.tracking_timeout = self.get_parameter('tracking_timeout').value
        self.min_detections_stable = self.get_parameter('min_detections_stable').value
        self.median_window = self.get_parameter('median_filter_window').value
        self.ray_percentile = self.get_parameter('ray_percentile').value
        self.min_valid_rays = self.get_parameter('min_valid_rays').value
        self.max_obj_dist = self.get_parameter('max_object_distance').value
        self.nav2_graph_filepath = self.get_parameter('nav2_graph_filepath').value
        self.convert_script_path = self.get_parameter('convert_script_path').value

        # ── Camera intrinsics (populated from CameraInfo) ───────
        self.fx: Optional[float] = None
        self.cx: Optional[float] = None
        self.img_width: int = 640
        self.cam_info_received = False

        # ── LiDAR config (populated from first scan) ───────────
        self.scan_angle_min: float = 0.0
        self.scan_angle_max: float = 0.0
        self.scan_angle_increment: float = 0.0
        self.scan_range_min: float = 0.0
        self.scan_range_max: float = 0.0
        self.scan_configured = False

        # ── State ───────────────────────────────────────────────
        self.last_scan: Optional[LaserScan] = None
        self.last_process_time: float = 0.0
        self.tracked_objects: List[TrackedObject] = []
        self.detection_count: int = 0

        # ── Semantic Map Manager ─────────────────────────────────
        self.map_manager = SemanticMapManager(
            objects_filepath=self.get_parameter('semantic_objects_filepath').value,
            route_graph_filepath=self.get_parameter('route_graph_filepath').value,
            output_graph_filepath=self.get_parameter('output_graph_filepath').value,
            match_distance=self.tracking_dist_thresh,
            edge_proximity=self.get_parameter('edge_proximity').value,
            logger=self.get_logger(),
        )

        # ── Route Server service client ──────────────────────────
        self._route_graph_client = self.create_client(
            SetRouteGraph, '/route_server/set_route_graph'
        )

        # ── Reset service for experimental protocol ──────────────
        self.create_service(
            Trigger,
            '~/reset_map',
            self._reset_map_cb
        )

        # ── Preîncarcă funcția de conversie ─────────────────────
        self._convert_geojson_fn = None
        self._load_convert_fn()

        # ── TF2 ─────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Subscribers ─────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self._camera_info_cb,
            1
        )

        self.create_subscription(
            LaserScan,
            self.get_parameter('lidar_topic').value,
            self._scan_cb,
            sensor_qos
        )

        self.create_subscription(
            Detection2DArray,
            self.get_parameter('detections_topic').value,
            self._detections_cb,
            sensor_qos
        )

        # ── Publishers ──────────────────────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray, '/semantic_markers', 10
        )

        self.point_pub = self.create_publisher(
            PointStamped, '/semantic_detections/point', 10
        )

        # ── Timere ──────────────────────────────────────────────
        self.create_timer(10.0, self._diagnostics_cb)

        save_interval = self.get_parameter('save_interval').value
        cleanup_interval = self.get_parameter('cleanup_interval').value
        self.create_timer(save_interval, self._save_maps_cb)
        self.create_timer(cleanup_interval, self._cleanup_cb)

        self.get_logger().info(
            f'SemanticLocalizer started | rate={self.max_rate}Hz | '
            f'conf>={self.min_confidence} | max_dist={self.max_obj_dist}m'
        )

    # ═══════════════════════════════════════════════════════════════
    # CONVERT SCRIPT LOADER
    # ═══════════════════════════════════════════════════════════════

    def _load_convert_fn(self):
        """Importă funcția convert_geojson din scriptul de conversie."""
        script_path = Path(self.convert_script_path)
        if not script_path.exists():
            self.get_logger().warn(
                f'Script conversie nu găsit: {script_path} — auto-reload dezactivat'
            )
            return
        try:
            script_dir = str(script_path.parent)
            if script_dir not in sys.path:
                sys.path.insert(0, script_dir)
            # Importă modulul după nume fișier
            import importlib.util
            spec = importlib.util.spec_from_file_location('convert_geojson_mod', script_path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._convert_geojson_fn = mod.convert_geojson
            self.get_logger().info(f'Funcție conversie încărcată din {script_path.name}')
        except Exception as e:
            self.get_logger().error(f'Nu am putut încărca scriptul de conversie: {e}')

    # ═══════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════

    def _camera_info_cb(self, msg: CameraInfo):
        """Extract camera intrinsics (only needs to run once)."""
        if self.cam_info_received:
            return
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.img_width = msg.width
        self.cam_info_received = True
        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx:.1f}, cx={self.cx:.1f}, '
            f'width={self.img_width}'
        )

    def _scan_cb(self, msg: LaserScan):
        """Cache latest LiDAR scan."""
        self.last_scan = msg
        if not self.scan_configured:
            self.scan_angle_min = msg.angle_min
            self.scan_angle_max = msg.angle_max
            self.scan_angle_increment = msg.angle_increment
            self.scan_range_min = msg.range_min
            self.scan_range_max = msg.range_max
            n_rays = len(msg.ranges)
            self.scan_configured = True
            self.get_logger().info(
                f'LiDAR configured: {n_rays} rays, '
                f'[{math.degrees(msg.angle_min):.1f}°, '
                f'{math.degrees(msg.angle_max):.1f}°], '
                f'increment={math.degrees(msg.angle_increment):.3f}°'
            )

    def _detections_cb(self, msg: Detection2DArray):
        """
        Main processing callback — triggered by each YOLO detection message.
        Rate-limited to max_process_rate_hz.
        """
        now = time.monotonic()
        if now - self.last_process_time < self.min_interval:
            return

        if not self.cam_info_received:
            return

        if self.last_scan is None:
            return

        self.last_process_time = now
        detections = msg.detections

        if len(detections) == 0:
            return

        semantic_objects: List[SemanticObject] = []
        ros_now = Time(seconds=0, nanoseconds=0)

        for det in detections:
            if len(det.results) == 0:
                continue

            confidence = det.results[0].hypothesis.score
            if confidence < self.min_confidence:
                continue

            class_id_str = det.results[0].hypothesis.class_id
            class_name = det.id if det.id else f'class_{class_id_str}'
            class_id = int(class_id_str) if class_id_str.isdigit() else -1

            bb_cx = det.bbox.center.position.x
            bb_cy = det.bbox.center.position.y
            bb_w = det.bbox.size_x
            bb_h = det.bbox.size_y

            if bb_w <= 0 or bb_h <= 0:
                continue

            bb_x_min = bb_cx - bb_w / 2.0
            bb_x_max = bb_cx + bb_w / 2.0

            angle_left = math.atan2(bb_x_min - self.cx, self.fx)
            angle_right = math.atan2(bb_x_max - self.cx, self.fx)

            lidar_angle_1 = -angle_right + self.cam_lidar_yaw_offset
            lidar_angle_2 = -angle_left + self.cam_lidar_yaw_offset

            lidar_angle_min = min(lidar_angle_1, lidar_angle_2)
            lidar_angle_max = max(lidar_angle_1, lidar_angle_2)

            distance, n_valid, center_angle = self._get_lidar_distance(
                lidar_angle_min, lidar_angle_max
            )

            if distance is None:
                continue

            if distance > self.max_obj_dist:
                continue

            x_laser = distance * math.cos(center_angle)
            y_laser = distance * math.sin(center_angle)

            point_map = self._transform_to_map(
                x_laser, y_laser, self.laser_frame, ros_now
            )

            if point_map is None:
                continue

            obj = SemanticObject(
                class_name=class_name,
                class_id=class_id,
                confidence=confidence,
                x_map=point_map[0],
                y_map=point_map[1],
                distance=distance,
                timestamp=ros_now.nanoseconds / 1e9,
                bbox_center_px=(bb_cx, bb_cy),
                bbox_size_px=(bb_w, bb_h),
                lidar_rays_used=n_valid,
                angular_width_deg=math.degrees(lidar_angle_max - lidar_angle_min),
            )
            semantic_objects.append(obj)

        if semantic_objects:
            self._update_tracking(semantic_objects)
            self._publish_markers(semantic_objects, ros_now)
            self.detection_count += len(semantic_objects)

            for det in semantic_objects:
                self.map_manager.update_object(
                    class_name=det.class_name,
                    class_id=det.class_id,
                    x=det.x_map,
                    y=det.y_map,
                    confidence=det.confidence,
                    distance=det.distance,
                    lidar_rays=det.lidar_rays_used,
                )

    # ═══════════════════════════════════════════════════════════════
    # LIDAR DISTANCE EXTRACTION
    # ═══════════════════════════════════════════════════════════════

    def _get_lidar_distance(
        self, angle_min: float, angle_max: float
    ) -> Tuple[Optional[float], int, float]:
        scan = self.last_scan
        ranges = np.array(scan.ranges, dtype=np.float32)
        n_rays = len(ranges)

        def normalize(a):
            while a < 0:
                a += 2 * math.pi
            while a >= 2 * math.pi:
                a -= 2 * math.pi
            return a

        a_min = normalize(angle_min)
        a_max = normalize(angle_max)

        idx_min = int((a_min - self.scan_angle_min) / self.scan_angle_increment)
        idx_max = int((a_max - self.scan_angle_min) / self.scan_angle_increment)

        idx_min = max(0, min(n_rays - 1, idx_min))
        idx_max = max(0, min(n_rays - 1, idx_max))

        if idx_min <= idx_max:
            selected = ranges[idx_min:idx_max + 1]
            center_idx = (idx_min + idx_max) // 2
        else:
            selected = np.concatenate([ranges[idx_min:], ranges[:idx_max + 1]])
            total = (n_rays - idx_min) + idx_max + 1
            center_idx = (idx_min + total // 2) % n_rays

        valid_mask = (selected > self.scan_range_min) & (selected < self.scan_range_max) & np.isfinite(selected)
        valid = selected[valid_mask]

        if len(valid) < self.min_valid_rays:
            return None, 0, 0.0

        distance = float(np.percentile(valid, self.ray_percentile))
        center_angle = self.scan_angle_min + center_idx * self.scan_angle_increment

        return distance, len(valid), center_angle

    # ═══════════════════════════════════════════════════════════════
    # TF2 TRANSFORM
    # ═══════════════════════════════════════════════════════════════

    def _transform_to_map(
        self, x: float, y: float, source_frame: str, stamp: Time
    ) -> Optional[Tuple[float, float]]:
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = stamp.to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0

        try:
            transformed = self.tf_buffer.transform(
                point, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return (transformed.point.x, transformed.point.y)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(
                f'TF transform failed ({source_frame}→{self.map_frame}): {e}',
                throttle_duration_sec=5.0
            )
            return None

    # ═══════════════════════════════════════════════════════════════
    # TRACKING
    # ═══════════════════════════════════════════════════════════════

    def _update_tracking(self, detections: List[SemanticObject]):
        current_time = time.monotonic()

        self.tracked_objects = [
            t for t in self.tracked_objects
            if (current_time - t.last_seen) < self.tracking_timeout
        ]

        for det in detections:
            matched = False
            for tracked in self.tracked_objects:
                if tracked.class_name != det.class_name:
                    continue
                dx = tracked.stable_x - det.x_map
                dy = tracked.stable_y - det.y_map
                dist = math.sqrt(dx * dx + dy * dy)
                if dist < self.tracking_dist_thresh:
                    tracked.positions.append((det.x_map, det.y_map))
                    tracked.confidences.append(det.confidence)
                    if len(tracked.positions) > self.median_window:
                        tracked.positions = tracked.positions[-self.median_window:]
                        tracked.confidences = tracked.confidences[-self.median_window:]
                    xs = [p[0] for p in tracked.positions]
                    ys = [p[1] for p in tracked.positions]
                    tracked.stable_x = float(np.median(xs))
                    tracked.stable_y = float(np.median(ys))
                    tracked.last_seen = current_time
                    tracked.detection_count += 1
                    matched = True
                    break

            if not matched:
                self.tracked_objects.append(TrackedObject(
                    class_name=det.class_name,
                    positions=[(det.x_map, det.y_map)],
                    confidences=[det.confidence],
                    last_seen=current_time,
                    stable_x=det.x_map,
                    stable_y=det.y_map,
                    detection_count=1,
                ))

    # ═══════════════════════════════════════════════════════════════
    # VISUALIZATION
    # ═══════════════════════════════════════════════════════════════

    def _publish_markers(
        self, detections: List[SemanticObject], stamp: Time
    ):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = stamp.to_msg()
        delete_marker.action = Marker.DELETEALL
        delete_marker.ns = 'semantic'
        marker_array.markers.append(delete_marker)

        for i, det in enumerate(detections):
            color = CLASS_COLORS.get(det.class_name, CLASS_COLORS['default'])

            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = stamp.to_msg()
            sphere.ns = 'semantic'
            sphere.id = i * 2
            sphere.type = Marker.CYLINDER
            sphere.action = Marker.ADD
            sphere.pose.position.x = det.x_map
            sphere.pose.position.y = det.y_map
            sphere.pose.position.z = 0.15
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            sphere.lifetime.sec = 2
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.map_frame
            text.header.stamp = stamp.to_msg()
            text.ns = 'semantic'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = det.x_map
            text.pose.position.y = det.y_map
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = (
                f'{det.class_name} ({det.confidence:.0%})\n'
                f'd={det.distance:.1f}m  rays={det.lidar_rays_used}'
            )
            text.lifetime.sec = 2
            marker_array.markers.append(text)

            ps = PointStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = stamp.to_msg()
            ps.point.x = det.x_map
            ps.point.y = det.y_map
            ps.point.z = 0.0
            self.point_pub.publish(ps)

        self.marker_pub.publish(marker_array)

    # ═══════════════════════════════════════════════════════════════
    # DIAGNOSTICS & MAP MANAGEMENT
    # ═══════════════════════════════════════════════════════════════

    def _reset_map_cb(self, request, response):
        """Service callback: clear all semantic objects and regenerate graph."""
        count = self.map_manager.reset_objects()
        self.tracked_objects.clear()

        # Force save empty map + regenerate graph + reload Route Server
        self._save_maps_cb()

        response.success = True
        response.message = f'Reset: cleared {count} objects, graph regenerated'
        self.get_logger().info(response.message)
        return response

    def _save_maps_cb(self):
        """Periodic save + conversie GeoJSON + reload Route Server."""
        saved = self.map_manager.save_objects()
        if not saved:
            return

        self.map_manager.save_annotated_graph()

        # ── Conversie semantic → nav2-compatibil ─────────────────
        if self._convert_geojson_fn is None:
            self.get_logger().warn('Funcție conversie indisponibilă, skip reload')
            return

        try:
            semantic_path = Path(self.get_parameter('output_graph_filepath').value)
            nav2_path = Path(self.nav2_graph_filepath)

            with open(semantic_path, 'r') as f:
                data = json.load(f)

            converted, nodes, edges = self._convert_geojson_fn(data)

            nav2_path.parent.mkdir(parents=True, exist_ok=True)
            with open(nav2_path, 'w') as f:
                json.dump(converted, f, indent=2)

            self.get_logger().info(
                f'GeoJSON convertit: {nodes} noduri, {edges} edges → {nav2_path.name}'
            )
        except Exception as e:
            self.get_logger().error(f'Conversie GeoJSON eșuată: {e}')
            return

        # ── Reload Route Server ───────────────────────────────────
        if not self._route_graph_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Route Server service indisponibil, skip reload')
            return

        req = SetRouteGraph.Request()
        req.graph_filepath = str(nav2_path)
        future = self._route_graph_client.call_async(req)
        future.add_done_callback(self._route_graph_reload_cb)

    def _route_graph_reload_cb(self, future):
        """Callback pentru rezultatul reload-ului grafului."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Route Server: graf reîncărcat cu succes')
            else:
                self.get_logger().warn('Route Server: reload eșuat (success=False)')
        except Exception as e:
            self.get_logger().error(f'Route Server reload exception: {e}')

    def _cleanup_cb(self):
        """Periodic TTL cleanup of expired dynamic objects."""
        self.map_manager.cleanup_expired()

    def _diagnostics_cb(self):
        """Periodic status logging."""
        n_tracked = len(self.tracked_objects)
        stable = sum(
            1 for t in self.tracked_objects
            if t.detection_count >= self.min_detections_stable
        )
        stats = self.map_manager.get_stats()
        self.get_logger().info(
            f'Semantic: {self.detection_count} det | '
            f'{n_tracked} tracked ({stable} stable) | '
            f'map: {stats["total_objects"]} persistent {stats["by_mobility"]} | '
            f'graph={stats["has_route_graph"]}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down — saving semantic maps...')
        node.map_manager.save_objects()
        node.map_manager.save_annotated_graph()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()