#!/usr/bin/env python3
"""
semantic_navigation_ultra.py - ULTRA OPTIMIZED + DEBUG
-------------------------------------------------------
Optimizări + debugging pentru troubleshooting
"""

import math
import numpy as np
import time
import json
from pathlib import Path
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, asdict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PointStamped
from std_msgs.msg import Int32MultiArray
from nav2_msgs.srv import DynamicEdges
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import tf2_geometry_msgs

# ===========================================================================
# CONSTANTE
# ===========================================================================
FX, CX = 687.53846, 308.10513
ANGLE_WINDOW_RAD = math.radians(4.0)
RANGE_MIN, RANGE_MAX = 0.10, 10.0
FALLBACK_RANGE = 1.5
FRAME_LASER, FRAME_MAP = 'base_laser', 'map'
MIN_SCORE = 0.50
SYNC_SLOP = 1.0  # Increased from 0.2

# Graph
NODES = {0: (1.097, 2.588), 1: (2.313, 3.215), 2: (3.334, 2.844),
         3: (4.353, 3.299), 4: (4.469, 2.454), 5: (5.513, 2.768)}

EDGES = {14: (2, 3), 15: (3, 2), 18: (3, 5), 19: (5, 3),
         16: (2, 4), 17: (4, 2), 20: (4, 5), 21: (5, 4),
         10: (0, 1), 11: (1, 0), 12: (1, 2), 13: (2, 1)}

ROUTE_EDGES = {'A': [14, 15, 18, 19], 'B': [16, 17, 20, 21]}

# CACHE pre-calculat pentru segmente
EDGE_SEGMENTS = {
    eid: (np.array(NODES[s]), np.array(NODES[e])) 
    for eid, (s, e) in EDGES.items()
}

# ===========================================================================
# TRACKING SIMPLU
# ===========================================================================

@dataclass
class TrackedObject:
    id: str
    label: str
    x: float
    y: float
    last_seen: float
    
    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(self.x - x, self.y - y)


class SimpleTracker:
    def __init__(self):
        self.objects: Dict[str, TrackedObject] = {}
        self.next_id = 0
    
    def update(self, label: str, x: float, y: float, timestamp: float) -> str:
        # Merge simplu: doar update sau adaugă
        for obj_id, obj in self.objects.items():
            if obj.label == label and obj.distance_to(x, y) < 0.3:
                obj.x, obj.y, obj.last_seen = x, y, timestamp
                return obj_id
        
        obj_id = f"{label}_{self.next_id:03d}"
        self.next_id += 1
        self.objects[obj_id] = TrackedObject(obj_id, label, x, y, timestamp)
        return obj_id
    
    def get_active(self, max_age: float = 10.0) -> List[TrackedObject]:
        current_time = time.time()
        return [obj for obj in self.objects.values() 
                if (current_time - obj.last_seen) < max_age]
    
    def cleanup(self, max_age: float = 30.0):
        current_time = time.time()
        to_remove = [oid for oid, obj in self.objects.items()
                    if (current_time - obj.last_seen) > max_age]
        for oid in to_remove:
            del self.objects[oid]


# ===========================================================================
# NOD ULTRA OPTIMIZAT
# ===========================================================================

class SemanticNavigationUltra(Node):
    
    def __init__(self):
        super().__init__('semantic_navigation_ultra')
        
        # Parametri
        self.min_score = 0.50
        self.angle_window = ANGLE_WINDOW_RAD
        self.fallback_range = FALLBACK_RANGE
        self.block_distance = 0.5
        self.persistence_time = 3.0
        
        # THROTTLE mai relaxat
        self.callback_interval = 1.0  # 3.3 Hz
        self.last_callback_time = 0.0
        
        self.tracker = SimpleTracker()
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE, depth=5)
        
        # Subscribers
        self.sub_det = Subscriber(self, Detection2DArray, '/mediapipe/detections', qos_profile=qos_sub)
        self.sub_scan = Subscriber(self, LaserScan, '/scan', qos_profile=qos_sub)
        
        # Increased queue_size and slop for better sync
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_det, self.sub_scan], 
            queue_size=20,  # Increased from 5
            slop=SYNC_SLOP
        )
        self.sync.registerCallback(self.callback)
        
        # Publishers
        self.pub_poses = self.create_publisher(PoseArray, '/semantic_objects', qos_pub)
        self.pub_blocked = self.create_publisher(Int32MultiArray, '/semantic_blocked_edges', qos_pub)
        
        # Service clients
        self.edges_client = self.create_client(DynamicEdges, '/route_server/DynamicEdgesScorer/adjust_edges')
        self.reroute_client = self.create_client(Trigger, '/route_server/ReroutingService/reroute')
        
        self.service_ready = False
        self.reroute_ready = False
        self.create_timer(2.0, self._check_services)
        
        # State
        self.blocked_edges: Set[int] = set()
        self.edge_block_times: Dict[int, float] = {}
        
        # Timer cleanup
        self.create_timer(5.0, self._timer_callback)
        
        # Stats
        self._n_callbacks = 0
        self._n_callbacks_throttled = 0
        self._n_localized = 0
        self._n_blocks = 0
        
        self.get_logger().info(
            '🚀 ULTRA OPTIMIZED MODE\n'
            f'  Callback throttle: {self.callback_interval}s (~{1.0/self.callback_interval:.1f} Hz)\n'
            f'  Sync slop: {SYNC_SLOP}s\n'
            f'  Block dist: {self.block_distance}m'
        )
    
    def _check_services(self):
        if not self.service_ready:
            if self.edges_client.wait_for_service(timeout_sec=0.1):
                self.service_ready = True
                self.get_logger().info('✓ DynamicEdges ready')
        if not self.reroute_ready:
            if self.reroute_client.wait_for_service(timeout_sec=0.1):
                self.reroute_ready = True
                self.get_logger().info('✓ Reroute ready')
    
    def callback(self, det_msg: Detection2DArray, scan_msg: LaserScan):
        # DEBUG: Log când primește callback
        self.get_logger().info(
            f'🔍 CALLBACK! dets={len(det_msg.detections)}',
            throttle_duration_sec=2.0
        )
        
        # THROTTLE
        current_time = time.time()
        if (current_time - self.last_callback_time) < self.callback_interval:
            self._n_callbacks_throttled += 1
            return
        self.last_callback_time = current_time
        
        self._n_callbacks += 1
        
        if not det_msg.detections:
            self.get_logger().info('No detections in callback', throttle_duration_sec=5.0)
            return
        
        self.get_logger().info(f'✅ PROCESSING callback #{self._n_callbacks}')
        
        ranges = self._preprocess_scan(scan_msg)
        timestamp = time.time()
        poses = []
        
        for det in det_msg.detections:
            if not det.results:
                continue
            
            score = det.results[0].hypothesis.score
            label = det.results[0].hypothesis.class_id
            
            self.get_logger().info(f'  Detection: {label} (score={score:.2f})')
            
            if score < self.min_score:
                self.get_logger().info(f'  Skipped: score {score:.2f} < {self.min_score}')
                continue
                
            if label != 'bottle':
                self.get_logger().info(f'  Skipped: label {label} != bottle')
                continue
            
            cx_px = det.bbox.center.position.x
            bearing_cam = math.atan2((cx_px - CX) / FX, 1.0)
            lidar_angle = (-bearing_cam) % (2.0 * math.pi)
            range_m, fallback = self._find_range(lidar_angle, ranges, scan_msg)
            
            self.get_logger().info(
                f'  Localization: bearing={math.degrees(bearing_cam):.1f}° '
                f'range={range_m:.2f}m {"[FALLBACK]" if fallback else ""}'
            )
            
            x_laser = range_m * math.cos(lidar_angle)
            y_laser = range_m * math.sin(lidar_angle)
            
            point = PointStamped()
            point.header.stamp = scan_msg.header.stamp
            point.header.frame_id = FRAME_LASER
            point.point.x, point.point.y, point.point.z = x_laser, y_laser, 0.0
            
            point_map = self._to_map(point)
            if point_map is None:
                self.get_logger().warn('  TF failed!')
                continue
            
            self._n_localized += 1
            x_map, y_map = point_map.point.x, point_map.point.y
            
            self.get_logger().info(f'  ✓ Localized in map: ({x_map:.2f}, {y_map:.2f})')
            
            obj_id = self.tracker.update(label, x_map, y_map, timestamp)
            
            pose = Pose()
            pose.position.x, pose.position.y = x_map, y_map
            pose.orientation.w = 1.0
            poses.append(pose)
        
        self.tracker.cleanup(max_age=30.0)
        self._check_and_block_edges(timestamp)
        
        if poses:
            arr = PoseArray()
            arr.header.stamp = scan_msg.header.stamp
            arr.header.frame_id = FRAME_MAP
            arr.poses = poses
            self.pub_poses.publish(arr)
        
        # Stats periodic
        if self._n_callbacks % 10 == 0:
            self.get_logger().info(
                f'📊 Stats: callbacks={self._n_callbacks} '
                f'throttled={self._n_callbacks_throttled} '
                f'localized={self._n_localized} '
                f'blocks={self._n_blocks}'
            )
    
    def _preprocess_scan(self, scan_msg: LaserScan) -> np.ndarray:
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        invalid = (~np.isfinite(ranges) | (ranges < scan_msg.range_min) | (ranges > scan_msg.range_max))
        ranges[invalid] = 0.0
        return ranges
    
    def _find_range(self, lidar_angle: float, ranges: np.ndarray, scan_msg: LaserScan) -> Tuple[float, bool]:
        idx_center = int((lidar_angle - scan_msg.angle_min) / scan_msg.angle_increment)
        idx_center = np.clip(idx_center, 0, len(ranges) - 1)
        
        half_win = max(1, int(self.angle_window / scan_msg.angle_increment))
        idx_start = max(0, idx_center - half_win)
        idx_end = min(len(ranges), idx_center + half_win + 1)
        
        window = ranges[idx_start:idx_end]
        valid = window[(window > RANGE_MIN) & (window < RANGE_MAX)]
        
        if len(valid) == 0:
            return self.fallback_range, True
        return float(np.min(valid)), False
    
    def _to_map(self, point: PointStamped):
        try:
            # Folosește direct lookup_transform și aplică manual
            transform = self.tf_buffer.lookup_transform(
                FRAME_MAP,
                FRAME_LASER,
                rclpy.time.Time(),  # Latest available
                timeout=Duration(seconds=0.5)
            )
            
            # Aplică transformarea manual
            from tf2_geometry_msgs import do_transform_point
            point_map = do_transform_point(point, transform)
            
            return point_map
            
        except Exception as e:
            self.get_logger().warn(f'TF error: {str(e)}', throttle_duration_sec=5.0)
            return None
    
    def _check_and_block_edges(self, timestamp: float):
        active_objects = self.tracker.get_active(max_age=10.0)
        if not active_objects:
            return
        
        self.get_logger().info(f'🔍 Checking {len(active_objects)} active objects against edges')
        
        edges_to_block = set()
        
        for obj in active_objects:
            obj_pos = np.array([obj.x, obj.y])
            
            for edge_id, (seg_start, seg_end) in EDGE_SEGMENTS.items():
                dist = self._point_to_segment_distance_fast(obj_pos, seg_start, seg_end)
                
                if dist < self.block_distance:
                    route = self._get_route_for_edge(edge_id)
                    if route:
                        edges_to_block.update(ROUTE_EDGES[route])
                        if edge_id not in self.blocked_edges:
                            self.get_logger().info(
                                f'🚫 [{obj.label}] blocks Route {route} '
                                f'(edge {edge_id}, dist={dist:.2f}m)'
                            )
        
        self._update_blocked_edges(edges_to_block, timestamp)
    
    def _point_to_segment_distance_fast(self, point: np.ndarray, 
                                       seg_start: np.ndarray, 
                                       seg_end: np.ndarray) -> float:
        seg_vec = seg_end - seg_start
        seg_len_sq = np.dot(seg_vec, seg_vec)
        
        if seg_len_sq == 0:
            return np.linalg.norm(point - seg_start)
        
        t = np.clip(np.dot(point - seg_start, seg_vec) / seg_len_sq, 0.0, 1.0)
        projection = seg_start + t * seg_vec
        return np.linalg.norm(point - projection)
    
    def _get_route_for_edge(self, edge_id: int) -> Optional[str]:
        if edge_id in ROUTE_EDGES['A']:
            return 'A'
        elif edge_id in ROUTE_EDGES['B']:
            return 'B'
        return None
    
    def _update_blocked_edges(self, new_blocked: Set[int], timestamp: float):
        to_close = new_blocked - self.blocked_edges
        if to_close:
            self.get_logger().info(f'🚫 Closing edges: {list(to_close)}')
            self._call_dynamic_edges(closed_edges=list(to_close))
            for edge in to_close:
                self.edge_block_times[edge] = timestamp
                self._n_blocks += 1
        
        to_open = self.blocked_edges - new_blocked
        if to_open:
            current_time = time.time()
            actually_open = [e for e in to_open 
                           if (current_time - self.edge_block_times.get(e, 0)) > self.persistence_time]
            if actually_open:
                self.get_logger().info(f'✅ Opening edges: {actually_open}')
                self._call_dynamic_edges(opened_edges=actually_open)
        
        self.blocked_edges = new_blocked
        
        msg = Int32MultiArray()
        msg.data = list(self.blocked_edges)
        self.pub_blocked.publish(msg)
    
    def _timer_callback(self):
        if not self.blocked_edges:
            return
        
        current_time = time.time()
        to_check = [e for e in self.blocked_edges 
                   if (current_time - self.edge_block_times.get(e, 0)) > self.persistence_time]
        
        if to_check:
            active_objects = self.tracker.get_active(max_age=self.persistence_time)
            if not active_objects:
                self._call_dynamic_edges(opened_edges=to_check)
                self.blocked_edges -= set(to_check)
                self.get_logger().info(f'✅ Auto-opened {len(to_check)} edges (no objects)')
    
    def _call_dynamic_edges(self, closed_edges: List[int] = None, opened_edges: List[int] = None):
        if not self.service_ready:
            self.get_logger().warn('DynamicEdges service not ready')
            return
        
        request = DynamicEdges.Request()
        if closed_edges:
            request.closed_edges = closed_edges
        if opened_edges:
            request.opened_edges = opened_edges
        
        self.edges_client.call_async(request)
        self._trigger_reroute()
    
    def _trigger_reroute(self):
        if not self.reroute_ready:
            return
        
        self.reroute_client.call_async(Trigger.Request())
        self.get_logger().info('🔄 Reroute triggered')
    
    def destroy_node(self):
        self.get_logger().info(
            f'Final stats: callbacks={self._n_callbacks} '
            f'throttled={self._n_callbacks_throttled} '
            f'localized={self._n_localized} '
            f'blocks={self._n_blocks}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigationUltra()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()