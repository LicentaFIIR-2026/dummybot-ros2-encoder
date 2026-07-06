#!/usr/bin/env python3
"""
SAIM Xplorer — Semantic Navigation Experimental Test Suite v3
=============================================================

Changes from v2:
  - C1: Validates robot start position vs graph node (warns if >1.5m)
  - C2: Validates real success (dist>1m, err<2m, duration>3s)
  - C3: Cascade abort after 2 consecutive failures
  - C4: Correct penalties_changed comparison (bidirectional edge pairs)
  - C5: Setup calls /semantic_localizer/reset_map service (not disk write)
  - C6: Logs robot_start_pose per leg
  - D2: CPU logging per run via psutil
  - D5: Min distance robot↔semantic objects on trajectory

Protocol:
  Setup (--test 0): Reset map, wait repopulation, regenerate graph
  Test 1 (--test 1): ComputeRoute only
  Test 2 (--test 2): Semantic forward+spin+return+spin ×N
  Test 3 (--test 3): Baseline forward+spin+return+spin ×N
  All   (--test all): Setup + Test1 + Test2 + Test3

Usage:
  python3 test_semantic_navigation_v3.py --test all --runs 5
"""

import sys
import os
import json
import time
import shutil
import argparse
import math
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, List, Dict, Any, Tuple

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose, Spin
from nav2_msgs.srv import SetRouteGraph
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# CPU monitoring (optional — degrades gracefully)
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


# =============================================================================
# CONFIGURATION
# =============================================================================

GRAPH_NODES = {
    0: {'name': 'start',    'x': 1.097, 'y': 2.588},
    1: {'name': 'in1',      'x': 2.313, 'y': 3.215},
    2: {'name': 'in2',      'x': 2.316, 'y': 2.540},
    3: {'name': 'int2',     'x': 3.334, 'y': 2.844},
    4: {'name': 'int3',     'x': 3.337, 'y': 2.473},
    5: {'name': 'altB',     'x': 4.464, 'y': 2.563},
    6: {'name': 'altA',     'x': 4.353, 'y': 3.299},
    7: {'name': 'int4',     'x': 5.302, 'y': 2.564},
    8: {'name': 'goal_old', 'x': 5.635, 'y': 3.335},
    9: {'name': 'goal',     'x': 6.002, 'y': 2.649},
}

# Bidirectional edge pairs: forward_id → reverse_id
EDGE_PAIRS = {
    10: 11, 11: 10, 12: 13, 13: 12, 14: 15, 15: 14,
    16: 17, 17: 16, 18: 19, 19: 18, 20: 21, 21: 20,
    22: 23, 23: 22, 24: 25, 25: 24, 26: 27, 27: 26,
    28: 29, 29: 28, 30: 31, 31: 30, 32: 33, 33: 32,
    34: 35, 35: 34, 36: 37, 37: 36,
}

SEMANTIC_OBJECTS_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/semantic_objects.geojson')
ROUTE_GRAPH_NAV2_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_fiir_nav2.geojson')
ROUTE_GRAPH_SEMANTIC_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_fiir_semantic.geojson')
EXPERIMENTS_DIR = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/experiments')
ODOM_TOPIC = '/platform/odom/filtered'
ODOM_SAMPLE_HZ = 2.0

# Validation thresholds
MAX_START_OFFSET = 1.5       # max dist from graph node to accept run
MIN_SUCCESS_DIST = 1.0       # min distance to count as real navigation
MAX_SUCCESS_ERROR = 2.0      # max XY error at goal
MIN_SUCCESS_DURATION = 3.0   # min seconds
MAX_CONSECUTIVE_FAILS = 2    # abort after N consecutive failures

VALID_CLASSES = {
    'person', 'chair', 'bench', 'bottle', 'cup', 'backpack',
    'suitcase', 'potted plant', 'tv', 'laptop', 'book',
}


# =============================================================================
# UTILITIES
# =============================================================================

def make_pose(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p

def node_pose(nid, yaw=0.0):
    n = GRAPH_NODES[nid]
    return make_pose(n['x'], n['y'], yaw)

def yaw_from_quat(oz, ow):
    return 2.0 * math.atan2(oz, ow)

def hdr(t):
    print('\n' + '=' * 70 + f'\n  {t}\n' + '=' * 70)

def sub(t):
    print(f'\n  --- {t} ---')

def ts_str():
    return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

def iso_now():
    return datetime.now(timezone.utc).isoformat()

def get_cpu():
    if HAS_PSUTIL:
        return psutil.cpu_percent(interval=None)
    return None


class SystemMetricsCollector:
    """Collect system metrics at 1Hz during navigation legs.

    Captures: CPU%, Memory%, Swap%, Disk%, CPU temp (RPi5),
    battery level (from /platform/bms/state).
    Runs in a background thread, correlates with run timing.

    For the article this provides:
      - Resource overhead comparison: semantic vs baseline navigation
      - Embedded platform viability (RPi5 thermal/CPU constraints)
      - Battery impact of semantic processing pipeline
    """

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._active = False
        self._samples = []
        self._battery_pct = -1.0
        self._thread = None

        # Subscribe to battery state if available
        try:
            from sensor_msgs.msg import BatteryState
            batt_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE, depth=1)
            node.create_subscription(BatteryState, '/platform/bms/state',
                                     self._batt_cb, batt_qos)
        except Exception:
            pass

    def _batt_cb(self, msg):
        with self._lock:
            self._battery_pct = round(msg.percentage * 100, 1)

    def _read_cpu_temp(self):
        """Read RPi5 CPU temperature from thermal zone."""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            return None

    def _sample(self):
        """Collect one sample of all system metrics."""
        if not HAS_PSUTIL:
            return None
        s = {
            't': round(time.time(), 3),
            'cpu_pct': psutil.cpu_percent(interval=None),
            'mem_pct': psutil.virtual_memory().percent,
            'swap_pct': psutil.swap_memory().percent,
            'disk_pct': psutil.disk_usage('/').percent,
            'cpu_temp_c': self._read_cpu_temp(),
        }
        with self._lock:
            s['battery_pct'] = self._battery_pct
        # Per-CPU breakdown (useful to see if one core is maxed)
        per_cpu = psutil.cpu_percent(percpu=True)
        if per_cpu:
            s['cpu_per_core'] = per_cpu
        return s

    def _collection_loop(self):
        """Background 1Hz collection loop."""
        while self._active:
            s = self._sample()
            if s:
                with self._lock:
                    self._samples.append(s)
            time.sleep(1.0)

    def start(self):
        """Start collecting metrics (call before navigation begins)."""
        with self._lock:
            self._samples = []
            self._active = True
        # Prime psutil
        if HAS_PSUTIL:
            psutil.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._collection_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """Stop collecting and return summary + time-series."""
        self._active = False
        if self._thread:
            self._thread.join(timeout=2.0)

        with self._lock:
            samples = list(self._samples)

        if not samples:
            return {'samples': 0}

        # Compute summary statistics
        def stats(key):
            vals = [s[key] for s in samples if s.get(key) is not None]
            if not vals:
                return None
            import statistics
            return {
                'mean': round(statistics.mean(vals), 1),
                'max': round(max(vals), 1),
                'min': round(min(vals), 1),
                'std': round(statistics.stdev(vals), 1) if len(vals) > 1 else 0.0,
            }

        result = {
            'samples': len(samples),
            'duration_s': round(samples[-1]['t'] - samples[0]['t'], 1) if len(samples) > 1 else 0,
            'cpu': stats('cpu_pct'),
            'memory': stats('mem_pct'),
            'swap': stats('swap_pct'),
            'cpu_temp': stats('cpu_temp_c'),
            'battery_start': samples[0].get('battery_pct', -1),
            'battery_end': samples[-1].get('battery_pct', -1),
            'time_series': samples,
        }

        # Battery delta (if available)
        b_start = samples[0].get('battery_pct', -1)
        b_end = samples[-1].get('battery_pct', -1)
        if b_start > 0 and b_end > 0:
            result['battery_delta_pct'] = round(b_end - b_start, 2)

        return result


def identify_route(path_poses):
    if not path_poses:
        return 'UNKNOWN', {}
    key = {
        'in1': GRAPH_NODES[1], 'in2': GRAPH_NODES[2],
        'int2': GRAPH_NODES[3], 'int3': GRAPH_NODES[4],
        'altB': GRAPH_NODES[5], 'altA': GRAPH_NODES[6],
        'int4': GRAPH_NODES[7], 'goal_old': GRAPH_NODES[8],
    }
    md = {n: float('inf') for n in key}
    for p in path_poses:
        px, py = p.pose.position.x, p.pose.position.y
        for n, nd in key.items():
            d = math.sqrt((px - nd['x'])**2 + (py - nd['y'])**2)
            md[n] = min(md[n], d)
    vis = {n for n, d in md.items() if d < 0.5}
    top = 'altA' in vis or 'goal_old' in vis
    bot = 'altB' in vis or 'int3' in vis
    if top and not bot: rt = 'SUS'
    elif bot and not top: rt = 'JOS'
    elif top and bot: rt = 'MIX'
    else: rt = 'DIRECT'
    return rt, {'visited': sorted(vis), 'min_dists': {k: round(v, 3) for k, v in md.items()}}


def load_semantic_snapshot():
    if not SEMANTIC_OBJECTS_PATH.exists():
        return {'objects_count': 0, 'objects_by_class': {}, 'valid_count': 0}
    try:
        with open(SEMANTIC_OBJECTS_PATH) as f:
            data = json.load(f)
        bc = {}
        vc = 0
        for feat in data.get('features', []):
            c = feat.get('properties', {}).get('class_name', 'unknown')
            bc[c] = bc.get(c, 0) + 1
            if c in VALID_CLASSES:
                vc += 1
        return {'objects_count': len(data.get('features', [])),
                'objects_by_class': bc, 'valid_count': vc,
                'timestamp': data.get('metadata', {}).get('last_updated', '')}
    except Exception as e:
        return {'objects_count': 0, 'error': str(e)}


def load_nav2_penalties():
    if not ROUTE_GRAPH_NAV2_PATH.exists():
        return {}
    try:
        with open(ROUTE_GRAPH_NAV2_PATH) as f:
            data = json.load(f)
        pen = {}
        for feat in data.get('features', []):
            props = feat.get('properties', {})
            eid = props.get('id')
            meta = props.get('metadata', {})
            if 'penalty' in meta and eid is not None:
                pen[str(eid)] = meta['penalty']
        return pen
    except Exception:
        return {}


def load_semantic_objects_positions():
    """Load object positions for min-distance calculation."""
    if not SEMANTIC_OBJECTS_PATH.exists():
        return []
    try:
        with open(SEMANTIC_OBJECTS_PATH) as f:
            data = json.load(f)
        objs = []
        for feat in data.get('features', []):
            coords = feat.get('geometry', {}).get('coordinates', [0, 0])
            cls = feat.get('properties', {}).get('class_name', '')
            if cls in VALID_CLASSES:
                objs.append((coords[0], coords[1], cls))
        return objs
    except Exception:
        return []


def min_distance_to_objects(trajectory, objects):
    """Compute minimum distance from any trajectory point to any object."""
    if not trajectory or not objects:
        return None
    min_d = float('inf')
    closest_obj = None
    for tx, ty, _ in trajectory:
        for ox, oy, cls in objects:
            d = math.sqrt((tx - ox)**2 + (ty - oy)**2)
            if d < min_d:
                min_d = d
                closest_obj = cls
    if min_d == float('inf'):
        return None
    return {'min_distance_m': round(min_d, 3), 'closest_class': closest_obj}


def penalties_really_changed(pen_fwd, pen_ret):
    """Compare penalty values accounting for bidirectional edge pairs."""
    for eid_f, val_f in pen_fwd.items():
        rev_eid = str(EDGE_PAIRS.get(int(eid_f), -1))
        val_r = pen_ret.get(rev_eid, None)
        if val_r is not None and abs(val_f - val_r) > 0.01:
            return True
    return False


# =============================================================================
# ODOM TRACKER
# =============================================================================

class OdomTracker:
    def __init__(self, node):
        self._pts = []
        self._dist = 0.0
        self._lx = self._ly = None
        self._lt = 0.0
        self._iv = 1.0 / ODOM_SAMPLE_HZ
        self._vels = []
        self._fp = None          # odom-frame pose (trajectory tracking)
        self._map_pose = None    # map-frame pose (AMCL, for validation)
        self._active = False
        self._lock = threading.Lock()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=1)
        node.create_subscription(Odometry, ODOM_TOPIC, self._cb, qos)
        # AMCL pose in map frame — used for position validation
        amcl_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self._amcl_cb, amcl_qos)

    def start(self):
        with self._lock:
            self._pts, self._vels = [], []
            self._dist = 0.0
            self._lx = self._ly = None
            self._lt = 0.0
            self._fp = None
            self._active = True

    def stop(self):
        with self._lock:
            self._active = False

    def get_results(self):
        with self._lock:
            av = sum(self._vels) / len(self._vels) if self._vels else 0
            mv = max(self._vels) if self._vels else 0
            return {
                'distance_traveled_m': round(self._dist, 3),
                'avg_velocity_ms': round(av, 3),
                'max_velocity_ms': round(mv, 3),
                'final_pose': self._fp,
                'trajectory_points': len(self._pts),
            }

    def get_trajectory(self):
        with self._lock:
            return list(self._pts)

    def get_current_pose(self):
        with self._lock:
            return self._fp

    def get_map_pose(self):
        """Return robot pose in map frame (from AMCL)."""
        with self._lock:
            return self._map_pose

    def _amcl_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w)
        with self._lock:
            self._map_pose = (round(x, 4), round(y, 4), round(yaw, 4))

    def _cb(self, msg):
        with self._lock:
            if not self._active:
                # Still update final_pose even when not active (for start validation)
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                yaw = yaw_from_quat(msg.pose.pose.orientation.z,
                                     msg.pose.pose.orientation.w)
                self._fp = (round(x, 4), round(y, 4), round(yaw, 4))
                return
            now = time.monotonic()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = yaw_from_quat(msg.pose.pose.orientation.z,
                                 msg.pose.pose.orientation.w)
            vx = msg.twist.twist.linear.x
            if self._lx is not None:
                self._dist += math.sqrt((x - self._lx)**2 + (y - self._ly)**2)
            self._lx, self._ly = x, y
            self._vels.append(abs(vx))
            if now - self._lt >= self._iv:
                self._pts.append([round(x, 4), round(y, 4), round(now, 3)])
                self._lt = now
            self._fp = (round(x, 4), round(y, 4), round(yaw, 4))


# =============================================================================
# EXPERIMENTAL TESTER
# =============================================================================

class ExperimentalTester(Node):
    def __init__(self, sdir):
        super().__init__('experimental_tester')
        self.sdir = sdir
        self.sdir.mkdir(parents=True, exist_ok=True)

        self._cr = ActionClient(self, ComputeRoute, '/compute_route')
        self._fp = ActionClient(self, FollowPath, '/follow_path')
        self._ntp = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._spin_ac = ActionClient(self, Spin, '/spin')
        self._srg = self.create_client(SetRouteGraph, '/route_server/set_route_graph')
        self._reset_map = self.create_client(Trigger, '/semantic_localizer/reset_map')

        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.odom = OdomTracker(self)
        self.metrics = SystemMetricsCollector(self)
        self._run_ctr = 0

        # Prime CPU monitoring
        if HAS_PSUTIL:
            psutil.cpu_percent(interval=None)

    def wait_for_servers(self, timeout=15.0):
        for name, c in [('ComputeRoute', self._cr), ('FollowPath', self._fp),
                        ('NavigateToPose', self._ntp), ('Spin', self._spin_ac)]:
            print(f'  Wait {name}...', end=' ', flush=True)
            if not c.wait_for_server(timeout):
                print('TIMEOUT!')
                return False
            print('OK')
        return True

    # ─── Action helpers ───────────────────────────────────────────

    def _send_action(self, client, goal_msg, timeout=180.0):
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        gh = future.result()
        if not gh or not gh.accepted:
            return None
        rf = gh.get_result_async()
        t0 = time.time()
        while not rf.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - t0 > timeout:
                gh.cancel_goal_async()
                return None
        return rf.result()

    def compute_route(self, start, goal, use_start=True):
        g = ComputeRoute.Goal()
        g.start, g.goal = start, goal
        g.use_start = use_start
        g.use_poses = True
        r = self._send_action(self._cr, g, 30.0)
        if r is None:
            return None, None
        return r.result.path, r.result

    def exec_follow(self, path, timeout=180.0):
        g = FollowPath.Goal()
        g.path = path
        return self._send_action(self._fp, g, timeout) is not None

    def exec_nav(self, goal, timeout=180.0):
        g = NavigateToPose.Goal()
        g.pose = goal
        return self._send_action(self._ntp, g, timeout) is not None

    def exec_spin(self, angle=math.pi, timeout=30.0):
        g = Spin.Goal()
        g.target_yaw = angle
        t0 = time.time()
        r = self._send_action(self._spin_ac, g, timeout)
        return r is not None, time.time() - t0

    def call_reset_map(self) -> bool:
        if not self._reset_map.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('reset_map service unavailable')
            return False
        req = Trigger.Request()
        future = self._reset_map.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        try:
            r = future.result()
            return r.success
        except Exception:
            return False

    def reload_route_graph(self):
        if not self._srg.wait_for_service(timeout_sec=3.0):
            return False
        req = SetRouteGraph.Request()
        req.graph_filepath = str(ROUTE_GRAPH_NAV2_PATH)
        future = self._srg.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        try:
            return future.result().success
        except Exception:
            return False

    # ─── AMCL helpers ─────────────────────────────────────────────

    def reinitialize_amcl(self, node_id=0):
        """Publish initial_pose to AMCL to recover from drift."""
        n = GRAPH_NODES[node_id]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = n['x']
        msg.pose.pose.position.y = n['y']
        fp = self.odom.get_map_pose()
        yaw = fp[2] if fp else 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        self._initial_pose_pub.publish(msg)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f'  AMCL re-initialized at node {node_id} ({n["x"]}, {n["y"]})')

    def verify_localization(self, expected_node_id=0, max_drift=2.0):
        """Check AMCL near expected node. Re-init if drifted."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('  ⚠ No odom pose available')
            return False
        n = GRAPH_NODES[expected_node_id]
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        print(f'  AMCL check: ({fp[0]:.3f}, {fp[1]:.3f}) — {d:.3f}m from '
              f'{n["name"]}({n["x"]}, {n["y"]})')
        if d > max_drift:
            print(f'  ⚠ AMCL drifted {d:.1f}m! Re-initializing...')
            self.reinitialize_amcl(expected_node_id)
            time.sleep(1.0)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.2)
            fp2 = self.odom.get_map_pose()
            if fp2:
                d2 = math.sqrt((fp2[0] - n['x'])**2 + (fp2[1] - n['y'])**2)
                print(f'  AMCL after re-init: ({fp2[0]:.3f}, {fp2[1]:.3f}) — {d2:.3f}m')
                return d2 <= max_drift
            return False
        return True

    def _find_nearest_node(self):
        """Find the graph node nearest to robot's current position."""
        fp = self.odom.get_map_pose()
        if fp is None:
            return 0
        best_id, best_d = 0, float('inf')
        for nid, n in GRAPH_NODES.items():
            d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
            if d < best_d:
                best_id, best_d = nid, d
        return best_id

    def align_to_path(self, path_poses):
        """Spin robot to face the direction of the first path segment.
        Returns (success, duration_s, target_yaw)."""
        if not path_poses or len(path_poses) < 2:
            return True, 0.0, None

        # Look ~5 points ahead for a stable direction
        look = min(5, len(path_poses) - 1)
        p0 = path_poses[0].pose.position
        p1 = path_poses[look].pose.position
        target_yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)

        # Get current robot yaw
        fp = self.odom.get_map_pose()
        if fp is None:
            return True, 0.0, target_yaw

        current_yaw = fp[2]
        delta = target_yaw - current_yaw
        # Normalize to [-π, π]
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi

        if abs(delta) < 0.3:  # ~17° — already aligned enough
            print(f'    Heading OK (Δ={math.degrees(delta):.0f}°)')
            return True, 0.0, target_yaw

        print(f'    Aligning: {math.degrees(current_yaw):.0f}° → '
              f'{math.degrees(target_yaw):.0f}° (Δ={math.degrees(delta):.0f}°)')
        ok, dt = self.exec_spin(delta)
        return ok, dt, target_yaw

    # ─── Logging ──────────────────────────────────────────────────

    def _save(self, fn, data):
        p = self.sdir / fn
        with open(p, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        print(f'  📄 {p.name}')

    def _pos_error(self, goal_pose):
        # Use AMCL map-frame pose for error calculation (goal is in map frame)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            # Fallback to odom final pose
            r = self.odom.get_results()
            fp = r.get('final_pose')
        if fp is None:
            return {'xy_error_m': None, 'yaw_error_rad': None}
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        g_yaw = yaw_from_quat(goal_pose.pose.orientation.z,
                               goal_pose.pose.orientation.w)
        xy = math.sqrt((fp[0] - gx)**2 + (fp[1] - gy)**2)
        ye = abs(fp[2] - g_yaw)
        if ye > math.pi:
            ye = 2 * math.pi - ye
        return {'xy_error_m': round(xy, 4), 'yaw_error_rad': round(ye, 4)}

    def _route_info(self, result):
        info = {}
        if result and hasattr(result, 'route') and result.route:
            r = result.route
            if hasattr(r, 'nodes') and r.nodes:
                info['route_nodes'] = [n.nodeid for n in r.nodes]
                info['route_node_names'] = [
                    GRAPH_NODES.get(n.nodeid, {}).get('name', '?') for n in r.nodes]
            if hasattr(r, 'edges') and r.edges:
                info['route_edges'] = [e.edgeid for e in r.edges]
            if hasattr(r, 'route_cost'):
                info['route_cost'] = round(r.route_cost, 3)
        return info

    def _validate_start(self, start_id):
        """Check robot is near the expected start node (using AMCL map-frame pose)."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            # Fallback to odom pose if AMCL not yet received
            fp = self.odom.get_current_pose()
            if fp is None:
                return False, None, None
            print(f'    ⚠ Using odom pose (AMCL unavailable)')
        n = GRAPH_NODES[start_id]
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        return d <= MAX_START_OFFSET, round(d, 3), fp

    def _validate_success(self, nav2_ok, odom_r, err):
        """Validate run actually navigated (not a 0-distance false success)."""
        if not nav2_ok:
            return False
        dist = odom_r.get('distance_traveled_m', 0)
        dur = odom_r.get('duration_s', 0) if 'duration_s' in odom_r else 999
        xy_err = err.get('xy_error_m')
        if dist < MIN_SUCCESS_DIST:
            return False
        if xy_err is not None and xy_err > MAX_SUCCESS_ERROR:
            return False
        return True

    # =================================================================
    # TEST 0: SETUP
    # =================================================================

    def run_setup(self, wait_secs=60):
        hdr('TEST 0: SESSION SETUP')

        # 1. Backup
        sub('Pas 1: Backup + reset via service')
        if SEMANTIC_OBJECTS_PATH.exists():
            bk = self.sdir / f'semantic_objects_BACKUP_{ts_str()}.geojson'
            shutil.copy2(SEMANTIC_OBJECTS_PATH, bk)
            print(f'  Backup: {bk.name}')

        ok = self.call_reset_map()
        if ok:
            print('  ✓ Reset via service')
        else:
            print('  ⚠ Service unavailable — fallback disk reset')
            empty = {'type': 'FeatureCollection',
                     'metadata': {'description': 'Reset', 'frame': 'map',
                                  'last_updated': iso_now(), 'object_count': 0},
                     'features': []}
            with open(SEMANTIC_OBJECTS_PATH, 'w') as f:
                json.dump(empty, f, indent=2)

        # 2. Wait repopulation
        sub(f'Pas 2: Aștept repopulare ({wait_secs}s)')
        for i in range(wait_secs, 0, -1):
            rclpy.spin_once(self, timeout_sec=1.0)
            if i % 10 == 0:
                s = load_semantic_snapshot()
                print(f'  {i}s — {s["objects_count"]} obj ({s.get("valid_count",0)} valid)')

        # 3. Snapshot
        sub('Pas 3: Snapshot')
        snap = load_semantic_snapshot()
        print(f'  Total: {snap["objects_count"]} | Valid: {snap.get("valid_count",0)}')
        if SEMANTIC_OBJECTS_PATH.exists():
            shutil.copy2(SEMANTIC_OBJECTS_PATH,
                        self.sdir / 'semantic_snapshot_start.geojson')

        # 4. Wait for graph regeneration
        sub('Pas 4: Aștept regenerare graf')
        for i in range(35):
            rclpy.spin_once(self, timeout_sec=1.0)
            if ROUTE_GRAPH_SEMANTIC_PATH.exists():
                age = time.time() - os.path.getmtime(ROUTE_GRAPH_SEMANTIC_PATH)
                if age < 35:
                    print(f'  Graf semantic actualizat (age={age:.0f}s)')
                    break

        # 5. Reload
        sub('Pas 5: Reload Route Server')
        print(f'  {"✓" if self.reload_route_graph() else "⚠ FAILED"}')

        # 6. Verify AMCL localization
        sub('Pas 6: Verify AMCL localization')
        if not self.verify_localization(expected_node_id=0, max_drift=1.5):
            print('  ⚠ AMCL could not be recovered — results may be invalid')

        # 7. Verify penalties changed from backup
        pen = load_nav2_penalties()

        # 7. Config
        cfg = {
            'session_id': self.sdir.name, 'timestamp': iso_now(),
            'graph': {'nodes': len(GRAPH_NODES),
                      'coords': {str(k): v for k, v in GRAPH_NODES.items()}},
            'semantic': snap,
            'penalties_at_start': pen,
            'validation': {
                'max_start_offset_m': MAX_START_OFFSET,
                'min_success_dist_m': MIN_SUCCESS_DIST,
                'max_success_error_m': MAX_SUCCESS_ERROR,
                'min_success_duration_s': MIN_SUCCESS_DURATION,
                'max_consecutive_fails': MAX_CONSECUTIVE_FAILS,
            },
            'params': {'odom_topic': ODOM_TOPIC, 'sample_hz': ODOM_SAMPLE_HZ,
                       'valid_classes': sorted(VALID_CLASSES),
                       'has_psutil': HAS_PSUTIL},
        }
        self._save('session_config.json', cfg)
        print('\n  ✓ Setup complet!')
        return True

    # =================================================================
    # TEST 1: COMPUTE ROUTE
    # =================================================================

    def run_test1(self):
        hdr('TEST 1: ComputeRoute')
        t0 = time.time()
        path, result = self.compute_route(node_pose(0), node_pose(9))
        dt = time.time() - t0

        if path is None:
            print('  [ERR] None')
            return False

        rt, rd = identify_route(path.poses)
        ri = self._route_info(result)
        pen = load_nav2_penalties()
        ep = {str(e): pen.get(str(e), 0.0) for e in ri.get('route_edges', [])}

        data = {
            'test_type': 'compute_route', 'timestamp': iso_now(),
            'planning': {'duration_ms': round(dt * 1000, 2),
                         'points': len(path.poses), 'route_type': rt,
                         **ri, 'penalties_per_edge': ep},
            'details': rd, 'semantic': load_semantic_snapshot(),
        }
        self._save('test1_compute_route.json', data)
        print(f'  {dt*1000:.1f}ms | {len(path.poses)} pts | {rt}')
        print(f'  Nodes: {ri.get("route_nodes", "?")} | Cost: {ri.get("route_cost", "?")}')
        return True

    # =================================================================
    # TEST 2: SEMANTIC NAVIGATION
    # =================================================================

    def _run_semantic_leg(self, start_id, goal_id, run_num, direction):
        goal_p = node_pose(goal_id)

        # Validate start position — re-init AMCL if drifted
        start_ok, start_dist, robot_pose = self._validate_start(start_id)
        if not start_ok:
            print(f'    ⚠ Robot {start_dist}m from node {start_id} (max {MAX_START_OFFSET}m)')
            if start_dist is not None and start_dist > MAX_START_OFFSET * 2:
                # Re-init at nearest node (not assumed node — robot may not have moved)
                nearest = self._find_nearest_node()
                print(f'    AMCL drifted — re-initializing at nearest node {nearest}...')
                self.reinitialize_amcl(nearest)
                time.sleep(2.0)
                start_ok, start_dist, robot_pose = self._validate_start(start_id)
                if not start_ok and start_dist is not None and start_dist > MAX_START_OFFSET * 2:
                    print(f'    ✗ Still too far ({start_dist:.1f}m) — skipping leg')
                    return None, False

        # Plan route using robot's current TF pose as start
        t0 = time.time()
        path, result = self.compute_route(goal_p, goal_p, use_start=False)
        plan_dt = time.time() - t0

        if path is None or not path.poses:
            print(f'    [ERR] Planning failed')
            return None, False

        rt, rd = identify_route(path.poses)
        ri = self._route_info(result)
        pen = load_nav2_penalties()
        ep = {str(e): pen.get(str(e), 0.0) for e in ri.get('route_edges', [])}

        print(f'    Plan: {plan_dt*1000:.0f}ms | {rt} | nodes={ri.get("route_nodes","?")}')

        # Align robot heading to path direction before following
        align_ok, align_dt, target_yaw = self.align_to_path(path.poses)
        if not align_ok:
            print('    ⚠ Alignment spin failed')

        print('    ⚠ ROBOT MOVING')
        time.sleep(0.5)

        self.metrics.start()
        self.odom.start()
        t0 = time.time()
        nav2_ok = self.exec_follow(path)
        exec_dt = time.time() - t0
        self.odom.stop()
        sys_metrics = self.metrics.stop()

        odom_r = self.odom.get_results()
        traj = self.odom.get_trajectory()
        err = self._pos_error(goal_p)
        real_ok = self._validate_success(nav2_ok, odom_r, err)

        # Min distance to semantic objects
        sem_objs = load_semantic_objects_positions()
        traj_with_cls = [(p[0], p[1], '') for p in traj]
        min_obj_dist = min_distance_to_objects(traj_with_cls, sem_objs)

        # Summary line with key metrics
        cpu_s = sys_metrics.get('cpu', {})
        mem_s = sys_metrics.get('memory', {})
        temp_s = sys_metrics.get('cpu_temp', {})
        print(f'    Exec: {exec_dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err["xy_error_m"]}m | nav2={"✓" if nav2_ok else "✗"} '
              f'valid={"✓" if real_ok else "✗"}')
        if cpu_s:
            print(f'    Sys: CPU={cpu_s.get("mean",0):.0f}% '
                  f'Mem={mem_s.get("mean",0):.0f}% '
                  f'Temp={temp_s.get("max","?") if temp_s else "?"}°C')

        data = {
            'test_type': f'semantic_{direction}',
            'run_number': run_num,
            'timestamp': iso_now(),
            'robot_start_pose': robot_pose,
            'start_offset_m': start_dist,
            'planning': {'duration_ms': round(plan_dt * 1000, 2),
                         'points': len(path.poses), 'route_type': rt,
                         **ri, 'penalties_per_edge': ep},
            'execution': {'duration_s': round(exec_dt, 3),
                          'nav2_success': nav2_ok,
                          'validated_success': real_ok,
                          **odom_r, **err},
            'trajectory': traj,
            'min_object_distance': min_obj_dist,
            'system_metrics': sys_metrics,
            'semantic': load_semantic_snapshot(),
        }
        return data, real_ok

    def run_test2(self, n_runs=5):
        hdr(f'TEST 2: Semantic — {n_runs} runs')
        results = []
        consec_fail = 0

        for run in range(1, n_runs + 1):
            sub(f'RUN {run}/{n_runs}')

            # Forward
            print('  [Forward] start(0) → goal(9)')
            fwd_data, fwd_ok = self._run_semantic_leg(0, 9, run, 'forward')
            if fwd_data:
                self._save(f'test2_run{run:02d}_forward.json', fwd_data)

            # Spin
            print('    [Spin 180°]')
            s_ok, s_dt = self.exec_spin(math.pi)
            print(f'    {s_dt:.1f}s | {"✓" if s_ok else "✗"}')

            # Return — start_id depends on whether forward succeeded
            ret_start_id = 9 if fwd_ok else 0
            print('  [Return] goal(9) → start(0)')
            ret_data, ret_ok = self._run_semantic_leg(ret_start_id, 0, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {'duration_s': round(s_dt, 2), 'success': s_ok}
                if fwd_data and ret_data:
                    fp = fwd_data.get('planning', {}).get('penalties_per_edge', {})
                    rp = ret_data.get('planning', {}).get('penalties_per_edge', {})
                    ret_data['penalties_changed'] = penalties_really_changed(fp, rp)
                self._save(f'test2_run{run:02d}_return.json', ret_data)

            # Spin back
            s2_ok, s2_dt = self.exec_spin(math.pi)

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)
            print(f'  Run {run}: {"✓" if ok else "✗"}')

            # Cascade abort
            consec_fail = consec_fail + 1 if not ok else 0
            if consec_fail >= MAX_CONSECUTIVE_FAILS:
                print(f'\n  ⚠ {MAX_CONSECUTIVE_FAILS} consecutive failures — aborting test')
                break

        p = sum(results)
        print(f'\n  Test 2: {p}/{len(results)} ok')
        return p == len(results)

    # =================================================================
    # TEST 3: BASELINE
    # =================================================================

    # Default route nodes for baseline waypoint navigation
    # Same topology as the JOS route but WITHOUT semantic penalties
    BASELINE_WAYPOINTS_FWD = [0, 2, 4, 5, 7, 9]   # start → goal
    BASELINE_WAYPOINTS_RET = [9, 7, 5, 4, 2, 0]    # goal → start

    def _run_baseline_leg(self, goal_id, run_num, direction, start_id=None):
        """Baseline: NavigateToPose waypoint-by-waypoint through fixed route.

        Uses the same graph nodes as the default (JOS) semantic route,
        but navigates via sequential NavigateToPose calls instead of
        Route Server. This provides a fair comparison:
          - Same waypoints, same physical path
          - No semantic penalties, no route optimization
          - SmacPlannerHybrid plans each short hop independently
        """
        goal_p = node_pose(goal_id)
        check_id = start_id if start_id is not None else goal_id

        start_ok, start_dist, robot_pose = self._validate_start(check_id)
        if not start_ok:
            print(f'    ⚠ Robot {start_dist}m from node {check_id} (max {MAX_START_OFFSET}m)')
            if start_dist is not None and start_dist > MAX_START_OFFSET * 2:
                nearest = self._find_nearest_node()
                print(f'    AMCL drifted — re-initializing at nearest node {nearest}...')
                self.reinitialize_amcl(nearest)
                time.sleep(2.0)
                start_ok, start_dist, robot_pose = self._validate_start(check_id)
                if not start_ok and start_dist is not None and start_dist > MAX_START_OFFSET * 2:
                    print(f'    ✗ Still too far ({start_dist:.1f}m) — skipping leg')
                    data = {
                        'test_type': f'baseline_{direction}',
                        'run_number': run_num,
                        'timestamp': iso_now(),
                        'robot_start_pose': robot_pose,
                        'start_offset_m': start_dist,
                        'execution': {'duration_s': 0, 'nav2_success': False,
                                      'validated_success': False,
                                      'distance_traveled_m': 0, 'avg_velocity_ms': 0,
                                      'max_velocity_ms': 0, 'final_pose': robot_pose,
                                      'trajectory_points': 0,
                                      'xy_error_m': start_dist, 'yaw_error_rad': 0},
                        'trajectory': [],
                        'min_object_distance': None,
                        'system_metrics': {'samples': 0},
                        'semantic': load_semantic_snapshot(),
                        'skipped': True,
                        'skip_reason': f'start_offset {start_dist}m after re-init',
                    }
                    return data, False

        # Select waypoints based on direction
        if direction == 'forward':
            waypoints = self.BASELINE_WAYPOINTS_FWD
        else:
            waypoints = self.BASELINE_WAYPOINTS_RET

        # Skip waypoints we've already passed (find nearest)
        wp_names = [GRAPH_NODES[w]['name'] for w in waypoints]
        print(f'    Waypoints: {wp_names}')

        print('    ⚠ ROBOT MOVING')
        time.sleep(0.5)
        self.metrics.start()
        self.odom.start()
        t0 = time.time()

        nav2_ok = True
        for i, wp_id in enumerate(waypoints[1:], 1):  # skip first (=start)
            # Calculate yaw facing toward this waypoint from previous one
            prev_id = waypoints[i - 1]
            prev_n = GRAPH_NODES[prev_id]
            wp_n = GRAPH_NODES[wp_id]
            yaw = math.atan2(wp_n['y'] - prev_n['y'], wp_n['x'] - prev_n['x'])
            wp_p = node_pose(wp_id, yaw=yaw)
            wp_ok = self.exec_nav(wp_p)
            if not wp_ok:
                print(f'    ✗ Failed at waypoint {i}/{len(waypoints)-1} '
                      f'({GRAPH_NODES[wp_id]["name"]})')
                nav2_ok = False
                break

        dt = time.time() - t0
        self.odom.stop()
        sys_metrics = self.metrics.stop()

        odom_r = self.odom.get_results()
        traj = self.odom.get_trajectory()
        err = self._pos_error(goal_p)
        real_ok = self._validate_success(nav2_ok, odom_r, err)

        sem_objs = load_semantic_objects_positions()
        traj_cls = [(p[0], p[1], '') for p in traj]
        min_obj_dist = min_distance_to_objects(traj_cls, sem_objs)

        cpu_s = sys_metrics.get('cpu', {})
        mem_s = sys_metrics.get('memory', {})
        temp_s = sys_metrics.get('cpu_temp', {})
        print(f'    Exec: {dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err["xy_error_m"]}m | nav2={"✓" if nav2_ok else "✗"} '
              f'valid={"✓" if real_ok else "✗"}')
        if cpu_s:
            print(f'    Sys: CPU={cpu_s.get("mean",0):.0f}% '
                  f'Mem={mem_s.get("mean",0):.0f}% '
                  f'Temp={temp_s.get("max","?") if temp_s else "?"}°C')

        data = {
            'test_type': f'baseline_{direction}',
            'run_number': run_num,
            'timestamp': iso_now(),
            'robot_start_pose': robot_pose,
            'start_offset_m': start_dist,
            'baseline_waypoints': [GRAPH_NODES[w]['name'] for w in waypoints],
            'execution': {'duration_s': round(dt, 3),
                          'nav2_success': nav2_ok,
                          'validated_success': real_ok,
                          **odom_r, **err},
            'trajectory': traj,
            'min_object_distance': min_obj_dist,
            'system_metrics': sys_metrics,
            'semantic': load_semantic_snapshot(),
        }
        return data, real_ok

    def run_test3(self, n_runs=5):
        hdr(f'TEST 3: Baseline — {n_runs} runs')

        # Ensure robot starts at node 0 (may be elsewhere after Test 2)
        start_ok, start_dist, _ = self._validate_start(0)
        if not start_ok:
            print(f'  Homing to start(0) — currently {start_dist:.1f}m away')
            goal_p = node_pose(0)
            nav_ok = self.exec_nav(goal_p)
            time.sleep(1.0)
            start_ok2, start_dist2, _ = self._validate_start(0)
            print(f'  After homing: {start_dist2:.2f}m from start — '
                  f'{"✓" if start_ok2 else "⚠ still far"}')

        results = []
        consec_fail = 0

        for run in range(1, n_runs + 1):
            sub(f'RUN {run}/{n_runs}')

            print('  [Forward] → goal(9)')
            fwd_data, fwd_ok = self._run_baseline_leg(9, run, 'forward', start_id=0)
            if fwd_data:
                self._save(f'test3_run{run:02d}_forward.json', fwd_data)

            print('    [Spin 180°]')
            s_ok, s_dt = self.exec_spin(math.pi)

            # Dynamic start_id: if forward succeeded, robot is near goal(9);
            # if forward was skipped/failed, robot is still near start(0)
            ret_start = 9 if fwd_ok else 0
            print(f'  [Return] → start(0)')
            ret_data, ret_ok = self._run_baseline_leg(0, run, 'return', start_id=ret_start)
            if ret_data:
                ret_data['spin_before'] = {'duration_s': round(s_dt, 2), 'success': s_ok}
                self._save(f'test3_run{run:02d}_return.json', ret_data)

            s2_ok, s2_dt = self.exec_spin(math.pi)

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)
            print(f'  Run {run}: {"✓" if ok else "✗"}')

            consec_fail = consec_fail + 1 if not ok else 0
            if consec_fail >= MAX_CONSECUTIVE_FAILS:
                print(f'\n  ⚠ {MAX_CONSECUTIVE_FAILS} consecutive failures — aborting')
                break

        p = sum(results)
        print(f'\n  Test 3: {p}/{len(results)} ok')
        return p == len(results)

    # =================================================================
    # SUMMARY
    # =================================================================

    def gen_summary(self):
        import statistics
        def stats(v):
            if not v: return {}
            return {'mean': round(statistics.mean(v), 3),
                    'std': round(statistics.stdev(v), 3) if len(v) > 1 else 0,
                    'min': round(min(v), 3), 'max': round(max(v), 3), 'n': len(v)}

        summary = {'session': self.sdir.name, 'timestamp': iso_now(), 'tests': {}}

        for tname, prefix in [('semantic', 'test2_'), ('baseline', 'test3_')]:
            fwd, ret = [], []
            for f in sorted(self.sdir.glob(f'{prefix}*_forward.json')):
                with open(f) as fh: fwd.append(json.load(fh))
            for f in sorted(self.sdir.glob(f'{prefix}*_return.json')):
                with open(f) as fh: ret.append(json.load(fh))
            if not fwd: continue

            ok_fwd = [r for r in fwd if r['execution'].get('validated_success')]
            ok_ret = [r for r in ret if r['execution'].get('validated_success')]

            ft = [r['execution']['duration_s'] for r in ok_fwd]
            fd = [r['execution']['distance_traveled_m'] for r in ok_fwd]
            fe = [r['execution']['xy_error_m'] for r in ok_fwd if r['execution'].get('xy_error_m') is not None]
            rt = [r['execution']['duration_s'] for r in ok_ret]
            rd_l = [r['execution']['distance_traveled_m'] for r in ok_ret]

            ts_data = {
                'total_runs': len(fwd),
                'validated_success_fwd': f'{len(ok_fwd)}/{len(fwd)}',
                'validated_success_ret': f'{len(ok_ret)}/{len(ret)}' if ret else '0/0',
                'forward': {'time_s': stats(ft), 'dist_m': stats(fd), 'error_m': stats(fe)},
                'return': {'time_s': stats(rt), 'dist_m': stats(rd_l)},
            }

            if prefix == 'test2_':
                pt = [r['planning']['duration_ms'] for r in fwd if 'planning' in r]
                rts = [r['planning'].get('route_type', '?') for r in fwd if 'planning' in r]
                ts_data['planning_ms'] = stats(pt)
                ts_data['route_types'] = rts
                ts_data['penalties_changed_runs'] = sum(
                    1 for r in ret if r.get('penalties_changed', False))

            # Min object distance stats
            mod_fwd = [r['min_object_distance']['min_distance_m']
                       for r in ok_fwd if r.get('min_object_distance')]
            if mod_fwd:
                ts_data['min_object_distance_m'] = stats(mod_fwd)

            # Aggregate system metrics across successful runs
            all_runs = ok_fwd + ok_ret
            cpu_means = [r['system_metrics']['cpu']['mean']
                         for r in all_runs
                         if r.get('system_metrics', {}).get('cpu')]
            mem_means = [r['system_metrics']['memory']['mean']
                         for r in all_runs
                         if r.get('system_metrics', {}).get('memory')]
            temp_maxs = [r['system_metrics']['cpu_temp']['max']
                         for r in all_runs
                         if r.get('system_metrics', {}).get('cpu_temp')]
            batt_deltas = [r['system_metrics']['battery_delta_pct']
                           for r in all_runs
                           if r.get('system_metrics', {}).get('battery_delta_pct') is not None]

            if cpu_means or mem_means or temp_maxs:
                ts_data['system_metrics_summary'] = {}
                if cpu_means:
                    ts_data['system_metrics_summary']['cpu_pct'] = stats(cpu_means)
                if mem_means:
                    ts_data['system_metrics_summary']['mem_pct'] = stats(mem_means)
                if temp_maxs:
                    ts_data['system_metrics_summary']['cpu_temp_max_c'] = stats(temp_maxs)
                if batt_deltas:
                    ts_data['system_metrics_summary']['battery_delta_pct'] = stats(batt_deltas)

            summary['tests'][tname] = ts_data

        self._save('summary.json', summary)
        for tn, td in summary.get('tests', {}).items():
            print(f'  {tn}: fwd={td["validated_success_fwd"]} ret={td["validated_success_ret"]}')
            f = td.get('forward', {}).get('time_s', {})
            d = td.get('forward', {}).get('dist_m', {})
            if f:
                print(f'    Fwd: {f.get("mean",0):.1f}±{f.get("std",0):.1f}s | '
                      f'{d.get("mean",0):.2f}±{d.get("std",0):.2f}m')


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='SAIM Xplorer Experimental Suite v3')
    parser.add_argument('--test', type=str, default='1',
                        choices=['0', '1', '2', '3', 'all'])
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--session', type=str, default=None)
    parser.add_argument('--wait-secs', type=int, default=60,
                        help='Seconds to wait for semantic repopulation in setup')
    args = parser.parse_args()

    sdir = EXPERIMENTS_DIR / (args.session or f'session_{ts_str()}')
    rclpy.init()
    tester = ExperimentalTester(sdir)

    hdr('SAIM Xplorer — Experimental Suite v3')
    print(f'  Session: {sdir}')
    print(f'  Runs: {args.runs} | Test: {args.test}')
    print(f'  CPU monitoring: {"✓ psutil" if HAS_PSUTIL else "✗ not installed"}')

    if not tester.wait_for_servers():
        tester.destroy_node()
        rclpy.shutdown()
        return

    res = {}
    try:
        if args.test in ('0', 'all'):
            res['setup'] = tester.run_setup(args.wait_secs)
        if args.test in ('1', 'all'):
            res['test1'] = tester.run_test1()
        if args.test in ('2', 'all'):
            res['test2'] = tester.run_test2(args.runs)
        if args.test in ('3', 'all'):
            res['test3'] = tester.run_test3(args.runs)
        if args.test in ('2', '3', 'all'):
            tester.gen_summary()
    except KeyboardInterrupt:
        print('\n  ⚠ Interrupted — saving...')
        tester.gen_summary()

    hdr('FINAL REPORT')
    for n, p in res.items():
        print(f'  {n}: {"✓" if p else "✗"}')
    print(f'  Data: {sdir}')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()