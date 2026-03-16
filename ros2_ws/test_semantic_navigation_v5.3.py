#!/usr/bin/env python3
"""
SAIM Xplorer — Semantic Navigation Experimental Test Suite v4
=============================================================

Changes from v3:
  - Removed Test 2 (static semantic) — superseded by adaptive Test 3
  - Renumbered: Setup=0, ComputeRoute=1, Baseline=2, Adaptive=3
  - Aggressive AMCL re-initialization at every session start + per-leg
  - Mandatory reset_map + graph regen in setup (clean penalties)
  - _validate_start spins slowly after AMCL reinit for convergence
  - Return leg uses _find_nearest_node() instead of assuming position
  - Pre-scan waits for nav2 graph file timestamp to confirm pipeline ran

Changes from v4 (heading fix):
  - Added spin_to_heading() method
  - TARGET_YAW_NODE0 / TARGET_YAW_NODE9 constants for required headings
  - run_test2 + run_test3: inter-leg spins now use spin_to_heading()
    instead of exec_spin(math.pi) so orientation is deterministic

Protocol:
  Setup (--test 0): Reset map, AMCL reinit, wait repopulation, regen graph
  Test 1 (--test 1): ComputeRoute only (shows routing decision)
  Test 2 (--test 2): Baseline NavigateToPose waypoints, no semantic
  Test 3 (--test 3): Adaptive semantic pre-scan + replanning on penalty change
  All   (--test all): Setup + Test1 + Test2 + Test3

Usage:
  python3 test_semantic_navigation_v4.py --test all --runs 5
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

try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False


# =============================================================================
# CONFIGURATION
# =============================================================================

GRAPH_NODES = {
    0: {'name': 'start',    'x': 1.000, 'y': 0.000},
    1: {'name': 'in1',      'x': 2.000, 'y': -0.150},
    2: {'name': 'in2',      'x': 1.700, 'y': 0.800},
    3: {'name': 'int2',     'x': 3.000, 'y': 0.500},
    4: {'name': 'int3',     'x': 3.000, 'y': -0.150},
    5: {'name': 'altB',     'x': 4.000, 'y': -0.150},
    6: {'name': 'altA',     'x': 4.700, 'y': 1.000},
    7: {'name': 'int4',     'x': 5.000, 'y': 0.220},
    8: {'name': 'goal_old', 'x': 5.700, 'y': 1.000},
    9: {'name': 'goal',     'x': 6.600, 'y': 0.700},
}

# Bidirectional edge pairs: forward_id -> reverse_id
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
ODOM_TOPIC = '/odom'
ODOM_SAMPLE_HZ = 2.0

# Validation thresholds
MAX_START_OFFSET = 2.0
MIN_SUCCESS_DIST = 1.0
MAX_SUCCESS_ERROR = 2.0
MIN_SUCCESS_DURATION = 3.0
MAX_CONSECUTIVE_FAILS = 2

# Test 3: Adaptive semantic navigation with replanning
REPLAN_CHECK_INTERVAL = 2.0
NODE_PROXIMITY_THRESH = 0.5
PRESCAN_WAIT_SECS = 35

VALID_CLASSES = {
    'person', 'chair', 'bench', 'bottle', 'cup', 'backpack',
    'suitcase', 'potted plant', 'tv', 'laptop', 'book',
}

# Target absolute headings (map frame) for each terminal node.
# Node 0 (start): looking toward the rest of the graph (forward direction)
#   quaternion z=0.03683782789909257, w=0.9993212568717214
TARGET_YAW_NODE0 = 2.0 * math.atan2(0.03683782789909257, 0.9993212568717214)

# Node 9 (goal): looking back toward start
#   quaternion z=-0.9995870503545529, w=0.028735496576264175
TARGET_YAW_NODE9 = 2.0 * math.atan2(-0.9995870503545529, 0.028735496576264175)


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
    """

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._active = False
        self._samples = []
        self._battery_pct = -1.0
        self._thread = None

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
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except Exception:
            return None

    def _sample(self):
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
        per_cpu = psutil.cpu_percent(percpu=True)
        if per_cpu:
            s['cpu_per_core'] = per_cpu
        return s

    def _collection_loop(self):
        while self._active:
            s = self._sample()
            if s:
                with self._lock:
                    self._samples.append(s)
            time.sleep(1.0)

    def start(self):
        with self._lock:
            self._samples = []
            self._active = True
        if HAS_PSUTIL:
            psutil.cpu_percent(interval=None)
        self._thread = threading.Thread(target=self._collection_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._active = False
        if self._thread:
            self._thread.join(timeout=2.0)

        with self._lock:
            samples = list(self._samples)

        if not samples:
            return {'samples': 0}

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
        self._fp = None
        self._map_pose = None
        self._active = False
        self._lock = threading.Lock()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        node.create_subscription(Odometry, ODOM_TOPIC, self._cb, qos)
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

    # ------------------------------------------------------------------ actions

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

    def spin_to_heading(self, target_yaw: float, timeout: float = 30.0):
        """Roteste robotul la un yaw absolut in map frame.

        Citeste yaw-ul curent din AMCL, calculeaza delta minima
        (normalizata la [-pi, pi]) si executa exec_spin cu acel delta.
        Spinurile de convergenta AMCL si pre-scan raman relative (exec_spin).

        Returns: (success, duration_s, delta_rad)
        """
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('    [spin_to_heading] no AMCL pose, fallback exec_spin(pi)')
            ok, dt = self.exec_spin(math.pi, timeout)
            return ok, dt, math.pi

        current_yaw = fp[2]
        delta = target_yaw - current_yaw

        # Normalizeaza la [-pi, pi]
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi

        print(f'    spin_to_heading: current={math.degrees(current_yaw):.1f}deg '
              f'target={math.degrees(target_yaw):.1f}deg '
              f'delta={math.degrees(delta):.1f}deg')

        if abs(delta) < 0.05:  # ~3deg deja aliniat
            print(f'    Heading OK, no spin needed')
            return True, 0.0, delta

        ok, dt = self.exec_spin(delta, timeout)
        return ok, dt, delta

    def exec_follow_with_replan(self, goal_pose, timeout=180.0):
        """Follow path with periodic replanning when penalties change."""
        t_global = time.time()
        reroute_log = []
        current_route = None
        segment = 0

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        while time.time() - t_global < timeout:
            segment += 1

            fp = self.odom.get_map_pose() or self.odom.get_current_pose()
            if fp:
                dist_to_goal = math.sqrt((fp[0] - goal_x)**2 + (fp[1] - goal_y)**2)
                if dist_to_goal < NODE_PROXIMITY_THRESH:
                    print(f'      [Seg {segment}] Already at goal ({dist_to_goal:.2f}m)')
                    reroute_log.append({
                        'segment': segment, 'timestamp': iso_now(),
                        'plan_ms': 0, 'route_type': 'AT_GOAL',
                        'route_nodes': [], 'route_edges': [],
                        'route_cost': 0, 'penalties': {},
                        'reason': 'at_goal', 'outcome': 'success',
                    })
                    return True, {
                        'segments': reroute_log, 'total_reroutes': segment - 1,
                        'final_route': current_route or [],
                        'outcome': 'success',
                    }

            plan_t0 = time.time()
            path, result = self.compute_route(goal_pose, goal_pose, use_start=False)
            plan_dt = time.time() - plan_t0

            if path is None or not path.poses:
                print(f'      [Seg {segment}] Route planning failed, fallback to NavigateToPose')
                nav_ok = self.exec_nav(goal_pose, timeout=max(30, timeout - (time.time() - t_global)))
                reroute_log.append({
                    'segment': segment, 'timestamp': iso_now(),
                    'plan_ms': round(plan_dt * 1000, 2),
                    'route_type': 'FALLBACK_NAV',
                    'route_nodes': [], 'route_edges': [],
                    'route_cost': 0, 'penalties': {},
                    'reason': 'fallback' if segment == 1 else 'reroute_fallback',
                    'outcome': 'success' if nav_ok else 'fail',
                })
                return nav_ok, {
                    'segments': reroute_log, 'total_reroutes': segment - 1,
                    'final_route': current_route or [],
                    'outcome': 'success' if nav_ok else 'plan_fail',
                }

            rt, rd = identify_route(path.poses)
            ri = self._route_info(result)
            pen_snapshot = load_nav2_penalties()
            route_edges = ri.get('route_edges', [])
            ep = {str(e): pen_snapshot.get(str(e), 0.0) for e in route_edges}
            route_nodes = ri.get('route_nodes', [])

            seg_info = {
                'segment': segment,
                'timestamp': iso_now(),
                'plan_ms': round(plan_dt * 1000, 2),
                'route_type': rt,
                'route_nodes': route_nodes,
                'route_edges': route_edges,
                'route_cost': ri.get('route_cost'),
                'penalties': ep,
                'reason': 'initial' if segment == 1 else 'reroute',
            }

            if current_route and route_nodes != current_route:
                seg_info['route_changed_from'] = current_route
                print(f'      REROUTE! {current_route} -> {route_nodes}')
            current_route = route_nodes

            print(f'      [Seg {segment}] {rt} nodes={route_nodes} cost={ri.get("route_cost","?")} '
                  f'({plan_dt*1000:.0f}ms)')

            fp_goal = FollowPath.Goal()
            fp_goal.path = path

            send_future = self._fp.send_goal_async(fp_goal)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
            goal_handle = send_future.result()
            if not goal_handle or not goal_handle.accepted:
                print(f'      [Seg {segment}] FollowPath rejected')
                seg_info['outcome'] = 'rejected'
                reroute_log.append(seg_info)
                return False, {'segments': reroute_log, 'total_reroutes': segment - 1,
                               'outcome': 'rejected'}

            result_future = goal_handle.get_result_async()
            last_check = time.time()
            did_reroute = False

            while not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.3)

                if time.time() - t_global > timeout:
                    goal_handle.cancel_goal_async()
                    seg_info['outcome'] = 'global_timeout'
                    reroute_log.append(seg_info)
                    return False, {'segments': reroute_log, 'total_reroutes': segment - 1,
                                   'outcome': 'timeout'}

                if time.time() - last_check >= REPLAN_CHECK_INTERVAL:
                    last_check = time.time()

                    rp = self.odom.get_map_pose() or self.odom.get_current_pose()
                    if rp:
                        d2g = math.sqrt((rp[0] - goal_x)**2 + (rp[1] - goal_y)**2)
                        if d2g < NODE_PROXIMITY_THRESH * 2:
                            continue

                    new_pen = load_nav2_penalties()
                    changed = False
                    for eid_s, old_val in ep.items():
                        new_val = new_pen.get(eid_s, old_val)
                        if abs(new_val - old_val) > 2.0:
                            changed = True
                            break

                    if changed:
                        print(f'      Penalties changed, cancelling + replanning')
                        goal_handle.cancel_goal_async()
                        for _ in range(10):
                            rclpy.spin_once(self, timeout_sec=0.2)
                            if result_future.done():
                                break
                        seg_info['outcome'] = 'rerouted'
                        reroute_log.append(seg_info)
                        did_reroute = True
                        break

            if did_reroute:
                time.sleep(0.5)
                continue

            fp_result = result_future.result()
            success = fp_result is not None
            seg_info['outcome'] = 'success' if success else 'fail'
            reroute_log.append(seg_info)

            return success, {
                'segments': reroute_log,
                'total_reroutes': segment - 1,
                'final_route': current_route,
                'outcome': 'success' if success else 'fail',
            }

        return False, {'segments': reroute_log, 'total_reroutes': segment - 1,
                       'outcome': 'timeout'}

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

    # ------------------------------------------------------------------- AMCL

    def reinitialize_amcl(self, node_id=0):
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

    def ensure_localization(self, expected_node_id=0):
        self.reinitialize_amcl(expected_node_id)
        time.sleep(1.0)

        print('    [AMCL convergence spin]')
        self.exec_spin(math.pi / 6, timeout=10.0)  # 30deg, mai putin deranjant
        time.sleep(1.0)

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('    No AMCL pose after reinit')
            return False

        n = GRAPH_NODES[expected_node_id]
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        print(f'    AMCL pose: ({fp[0]:.2f}, {fp[1]:.2f}), {d:.2f}m from '
            f'{n["name"]} ({n["x"]}, {n["y"]})')

        if d > 1.5:
            print('    Still drifted, retry with larger spin')
            self.reinitialize_amcl(expected_node_id)
            self.exec_spin(math.pi / 3, timeout=15.0)  # 60deg max la retry
            time.sleep(1.5)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.2)
            fp = self.odom.get_map_pose()
            if fp:
                d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
                print(f'    AMCL retry: {d:.2f}m from {n["name"]}')

        converged = d <= 1.5 if fp else False

        # Dupa convergenta AMCL, realiniaza la heading-ul nodului
        if converged:
            target = TARGET_YAW_NODE0 if expected_node_id == 0 else TARGET_YAW_NODE9
            print(f'    [Post-reinit realign to node {expected_node_id} heading]')
            self.spin_to_heading(target, timeout=15.0)

        return converged

    def verify_localization(self, expected_node_id=0, max_drift=2.0):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            print('  No odom pose available')
            return False
        n = GRAPH_NODES[expected_node_id]
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        print(f'  AMCL check: ({fp[0]:.3f}, {fp[1]:.3f}), {d:.3f}m from '
              f'{n["name"]}({n["x"]}, {n["y"]})')
        if d > max_drift:
            print(f'  AMCL drifted {d:.1f}m! Re-initializing...')
            self.reinitialize_amcl(expected_node_id)
            time.sleep(1.0)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.2)
            fp2 = self.odom.get_map_pose()
            if fp2:
                d2 = math.sqrt((fp2[0] - n['x'])**2 + (fp2[1] - n['y'])**2)
                print(f'  AMCL after re-init: ({fp2[0]:.3f}, {fp2[1]:.3f}), {d2:.3f}m')
                return d2 <= max_drift
            return False
        return True

    def _find_nearest_node(self):
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
        if not path_poses or len(path_poses) < 2:
            return True, 0.0, None
        look = min(5, len(path_poses) - 1)
        p0 = path_poses[0].pose.position
        p1 = path_poses[look].pose.position
        target_yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
        fp = self.odom.get_map_pose()
        if fp is None:
            return True, 0.0, target_yaw
        current_yaw = fp[2]
        delta = target_yaw - current_yaw
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        if abs(delta) < 0.3:
            print(f'    Heading OK (delta={math.degrees(delta):.0f}deg)')
            return True, 0.0, target_yaw
        print(f'    Aligning: {math.degrees(current_yaw):.0f}deg -> '
              f'{math.degrees(target_yaw):.0f}deg (delta={math.degrees(delta):.0f}deg)')
        ok, dt = self.exec_spin(delta)
        return ok, dt, target_yaw

    # ----------------------------------------------------------------- logging

    def _save(self, fn, data):
        p = self.sdir / fn
        with open(p, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        print(f'  {p.name}')

    def _pos_error(self, goal_pose):
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
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
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.2)
        fp = self.odom.get_map_pose()
        if fp is None:
            print(f'    AMCL unavailable, running ensure_localization')
            self.ensure_localization(start_id)
            fp = self.odom.get_map_pose()
            if fp is None:
                fp = self.odom.get_current_pose()
                if fp is None:
                    return False, None, None
                print(f'    Still using odom pose after reinit')
        n = GRAPH_NODES[start_id]
        d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        if d > MAX_START_OFFSET:
            print(f'    Robot {d:.2f}m from node {start_id}, running ensure_localization')
            self.ensure_localization(start_id)
            fp = self.odom.get_map_pose() or self.odom.get_current_pose()
            if fp:
                d = math.sqrt((fp[0] - n['x'])**2 + (fp[1] - n['y'])**2)
        return d <= MAX_START_OFFSET, round(d, 3), fp

    def _validate_success(self, nav2_ok, odom_r, err):
        if not nav2_ok:
            return False
        dist = odom_r.get('distance_traveled_m', 0)
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

        sub('Pas 1: Backup + reset via service')
        if SEMANTIC_OBJECTS_PATH.exists():
            bk = self.sdir / f'semantic_objects_BACKUP_{ts_str()}.geojson'
            shutil.copy2(SEMANTIC_OBJECTS_PATH, bk)
            print(f'  Backup: {bk.name}')

        ok = self.call_reset_map()
        if ok:
            print('  Reset via service OK')
        else:
            print('  Service unavailable, fallback disk reset')
            empty = {'type': 'FeatureCollection',
                     'metadata': {'description': 'Reset', 'frame': 'map',
                                  'last_updated': iso_now(), 'object_count': 0},
                     'features': []}
            with open(SEMANTIC_OBJECTS_PATH, 'w') as f:
                json.dump(empty, f, indent=2)

        sub(f'Pas 2: Astept repopulare ({wait_secs}s)')
        for i in range(wait_secs, 0, -1):
            rclpy.spin_once(self, timeout_sec=1.0)
            if i % 10 == 0:
                s = load_semantic_snapshot()
                print(f'  {i}s, {s["objects_count"]} obj ({s.get("valid_count",0)} valid)')

        sub('Pas 3: Snapshot')
        snap = load_semantic_snapshot()
        print(f'  Total: {snap["objects_count"]} | Valid: {snap.get("valid_count",0)}')
        if SEMANTIC_OBJECTS_PATH.exists():
            shutil.copy2(SEMANTIC_OBJECTS_PATH,
                        self.sdir / 'semantic_snapshot_start.geojson')

        sub('Pas 4: Astept regenerare graf')
        for i in range(35):
            rclpy.spin_once(self, timeout_sec=1.0)
            if ROUTE_GRAPH_SEMANTIC_PATH.exists():
                age = time.time() - os.path.getmtime(ROUTE_GRAPH_SEMANTIC_PATH)
                if age < 35:
                    print(f'  Graf semantic actualizat (age={age:.0f}s)')
                    break

        sub('Pas 5: Reload Route Server')
        print(f'  {"OK" if self.reload_route_graph() else "FAILED"}')

        sub('Pas 6: AMCL localization')
        if not self.ensure_localization(expected_node_id=0):
            print('  AMCL could not be recovered, results may be invalid')

        pen = load_nav2_penalties()

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
            'target_headings': {
                'node0_yaw_rad': TARGET_YAW_NODE0,
                'node9_yaw_rad': TARGET_YAW_NODE9,
            },
        }
        self._save('session_config.json', cfg)
        print('\n  Setup complet!')
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
    # TEST 2: BASELINE
    # =================================================================

    def _run_baseline_leg(self, start_id, goal_id, run_num, direction):
        """Baseline: single NavigateToPose, no semantic routing."""
        goal_p = node_pose(goal_id)

        start_ok, start_dist, robot_pose = self._validate_start(start_id)
        if not start_ok:
            print(f'    Robot {start_dist}m from node {start_id} after recovery, skipping')
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
                'skip_reason': f'start_offset {start_dist}m after recovery',
            }
            return data, False

        print(f'    NavigateToPose -> node {goal_id} ({GRAPH_NODES[goal_id]["name"]})')
        print('    ROBOT MOVING')
        time.sleep(0.5)

        self.metrics.start()
        self.odom.start()
        t0 = time.time()
        nav2_ok = self.exec_nav(goal_p)
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
        print(f'    Exec: {dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err["xy_error_m"]}m | nav2={"OK" if nav2_ok else "FAIL"} '
              f'valid={"OK" if real_ok else "FAIL"}')
        if cpu_s:
            mem_s = sys_metrics.get('memory', {})
            temp_s = sys_metrics.get('cpu_temp', {})
            print(f'    Sys: CPU={cpu_s.get("mean",0):.0f}% '
                  f'Mem={mem_s.get("mean",0):.0f}% '
                  f'Temp={temp_s.get("max","?") if temp_s else "?"}C')

        data = {
            'test_type': f'baseline_{direction}',
            'run_number': run_num,
            'timestamp': iso_now(),
            'robot_start_pose': robot_pose,
            'start_offset_m': start_dist,
            'timing': {
                'prescan_duration_s': 0.0,
                'navigation_duration_s': round(dt, 3),
                'total_duration_s': round(dt, 3),
            },
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

    def run_test2(self, n_runs=5):
        hdr(f'TEST 2: Baseline, {n_runs} runs')
        results = []
        consec_fail = 0

        for run in range(1, n_runs + 1):
            sub(f'RUN {run}/{n_runs}')

            print('  [Forward] start(0) -> goal(9)')
            fwd_data, fwd_ok = self._run_baseline_leg(0, 9, run, 'forward')
            if fwd_data:
                self._save(f'test2_run{run:02d}_forward.json', fwd_data)

            # Spin la heading absolut al nodului 9 (spre start)
            print('    [Spin -> node 9 heading]')
            s_ok, s_dt, s_delta = self.spin_to_heading(TARGET_YAW_NODE9)
            print(f'    {s_dt:.1f}s | delta={math.degrees(s_delta):.1f}deg | '
                  f'{"OK" if s_ok else "FAIL"}')

            if fwd_ok:
                ret_start = 9
            else:
                ret_start = self._find_nearest_node()
                print(f'    Forward failed, nearest node for return: {ret_start}')

            print(f'  [Return] -> start(0)')
            ret_data, ret_ok = self._run_baseline_leg(ret_start, 0, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {'duration_s': round(s_dt, 2),
                                           'success': s_ok,
                                           'delta_deg': round(math.degrees(s_delta), 1)}
                self._save(f'test2_run{run:02d}_return.json', ret_data)

            # Spin la heading absolut al nodului 0 (spre goal) pentru urmatoarea iteratie
            s2_ok, s2_dt, s2_delta = self.spin_to_heading(TARGET_YAW_NODE0)
            print(f'    [Spin -> node 0 heading] {s2_dt:.1f}s | '
                  f'delta={math.degrees(s2_delta):.1f}deg | {"OK" if s2_ok else "FAIL"}')

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)
            print(f'  Run {run}: {"OK" if ok else "FAIL"}')

            consec_fail = consec_fail + 1 if not ok else 0
            if consec_fail >= MAX_CONSECUTIVE_FAILS:
                print(f'\n  {MAX_CONSECUTIVE_FAILS} consecutive failures, aborting')
                break

        p = sum(results)
        print(f'\n  Test 2: {p}/{len(results)} ok')
        return p == len(results)

    # =================================================================
    # TEST 3: ADAPTIVE SEMANTIC
    # =================================================================

    def _prescan(self):
        print('    [Pre-scan] 360deg rotation for object detection...')

        nav2_ts_before = 0
        if ROUTE_GRAPH_NAV2_PATH.exists():
            nav2_ts_before = os.path.getmtime(ROUTE_GRAPH_NAV2_PATH)

        ok1, dt1 = self.exec_spin(math.pi, timeout=90.0)
        ok2, dt2 = self.exec_spin(math.pi, timeout=90.0)
        print(f'    Scan: {dt1 + dt2:.1f}s total | {"OK" if ok1 and ok2 else "WARN"}')

        snap_before = load_semantic_snapshot()
        print(f'    Objects before wait: {snap_before.get("objects_count", 0)} '
            f'(valid: {snap_before.get("valid_count", 0)})')

        print(f'    Waiting {PRESCAN_WAIT_SECS}s for semantic pipeline...')
        graph_updated = False
        for i in range(PRESCAN_WAIT_SECS, 0, -1):
            rclpy.spin_once(self, timeout_sec=1.0)
            if i % 10 == 0:
                s = load_semantic_snapshot()
                nav2_ts_now = os.path.getmtime(ROUTE_GRAPH_NAV2_PATH) if ROUTE_GRAPH_NAV2_PATH.exists() else 0
                if nav2_ts_now > nav2_ts_before:
                    graph_updated = True
                tag = 'graph ok' if graph_updated else 'waiting'
                print(f'      {i}s, {s.get("objects_count",0)} obj '
                    f'(valid: {s.get("valid_count",0)}) | {tag}')

        if not graph_updated:
            print(f'    Nav2 graph NOT updated, pipeline may be broken!')
        else:
            print(f'    Nav2 graph confirmed updated')

        snap_after = load_semantic_snapshot()
        print(f'    Objects after wait: {snap_after.get("objects_count", 0)} '
            f'(valid: {snap_after.get("valid_count", 0)})')

        self.reload_route_graph()
        time.sleep(1.0)

        # Realiniaza la heading corect dupa 360 (spinul acumuleaza eroare)
        print('    [Post-scan realign]')
        align_ok, align_dt, align_delta = self.spin_to_heading(TARGET_YAW_NODE0)
        print(f'    Realign: {align_dt:.1f}s | delta={math.degrees(align_delta):.1f}deg | '
            f'{"OK" if align_ok else "WARN"}')

        return snap_after

    def _run_adaptive_leg(self, start_id, goal_id, run_num, direction):
        """Adaptive semantic leg: pre-scan + follow with replanning."""
        goal_p = node_pose(goal_id)

        start_ok, start_dist, robot_pose = self._validate_start(start_id)
        if not start_ok:
            print(f'    Robot {start_dist}m from node {start_id} after recovery, skipping')
            return None, False

        prescan_snap = None
        prescan_duration = 0.0
        if direction == 'forward':
            t_prescan = time.time()
            prescan_snap = self._prescan()
            prescan_duration = time.time() - t_prescan

        pen_before = load_nav2_penalties()

        print('    ROBOT MOVING (adaptive)')
        time.sleep(0.5)

        self.metrics.start()
        self.odom.start()
        t0 = time.time()
        nav2_ok, replan_log = self.exec_follow_with_replan(goal_p)
        nav_dt = time.time() - t0
        self.odom.stop()
        sys_metrics = self.metrics.stop()

        odom_r = self.odom.get_results()
        traj = self.odom.get_trajectory()
        err = self._pos_error(goal_p)
        real_ok = self._validate_success(nav2_ok, odom_r, err)

        sem_objs = load_semantic_objects_positions()
        traj_cls = [(p[0], p[1], '') for p in traj]
        min_obj_dist = min_distance_to_objects(traj_cls, sem_objs)

        pen_after = load_nav2_penalties()

        segments = replan_log.get('segments', [])
        initial_route = segments[0].get('route_nodes', []) if segments else []
        final_route = segments[-1].get('route_nodes', []) if segments else []
        initial_route_type = segments[0].get('route_type', '?') if segments else '?'
        planning_total_ms = sum(s.get('plan_ms', 0) for s in segments)
        n_reroutes = replan_log.get('total_reroutes', 0)
        route_changed = initial_route != final_route

        print(f'    Exec: {nav_dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err.get("xy_error_m","?")}m | reroutes={n_reroutes} | '
              f'nav2={"OK" if nav2_ok else "FAIL"} valid={"OK" if real_ok else "FAIL"}')

        data = {
            'test_type': f'adaptive_{direction}',
            'run_number': run_num,
            'timestamp': iso_now(),
            'robot_start_pose': robot_pose,
            'start_offset_m': start_dist,
            'replanning': replan_log,
            'route_analysis': {
                'initial_route': initial_route,
                'final_route': final_route,
                'initial_route_type': initial_route_type,
                'route_changed': route_changed,
                'planning_total_ms': planning_total_ms,
                'n_segments': len(segments),
            },
            'timing': {
                'prescan_duration_s': round(prescan_duration, 2),
                'navigation_duration_s': round(nav_dt, 3),
                'total_duration_s': round(prescan_duration + nav_dt, 3),
            },
            'execution': {'duration_s': round(nav_dt, 3),
                          'nav2_success': nav2_ok,
                          'validated_success': real_ok,
                          **odom_r, **err},
            'trajectory': traj,
            'min_object_distance': min_obj_dist,
            'penalties_before': pen_before,
            'penalties_after': pen_after,
            'system_metrics': sys_metrics,
            'semantic': load_semantic_snapshot(),
        }
        if prescan_snap is not None:
            data['prescan_snapshot'] = prescan_snap
        return data, real_ok

    def run_test3(self, n_runs=5):
        hdr(f'TEST 3: Adaptive Semantic, {n_runs} runs')
        results = []
        consec_fail = 0

        for run in range(1, n_runs + 1):
            sub(f'RUN {run}/{n_runs}')

            print('  [Forward adaptive] start(0) -> goal(9)')
            fwd_data, fwd_ok = self._run_adaptive_leg(0, 9, run, 'forward')
            if fwd_data:
                self._save(f'test3_run{run:02d}_forward.json', fwd_data)

            # Spin la heading absolut al nodului 9 (spre start)
            print('    [Spin -> node 9 heading]')
            s_ok, s_dt, s_delta = self.spin_to_heading(TARGET_YAW_NODE9)
            print(f'    {s_dt:.1f}s | delta={math.degrees(s_delta):.1f}deg | '
                  f'{"OK" if s_ok else "FAIL"}')

            if fwd_ok:
                ret_start_id = 9
            else:
                ret_start_id = self._find_nearest_node()
                print(f'    Forward failed, nearest node for return: {ret_start_id}')

            print('  [Return adaptive] goal(9) -> start(0)')
            ret_data, ret_ok = self._run_adaptive_leg(ret_start_id, 0, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {'duration_s': round(s_dt, 2),
                                           'success': s_ok,
                                           'delta_deg': round(math.degrees(s_delta), 1)}
                self._save(f'test3_run{run:02d}_return.json', ret_data)

            # Spin la heading absolut al nodului 0 (spre goal) pentru urmatoarea iteratie
            s2_ok, s2_dt, s2_delta = self.spin_to_heading(TARGET_YAW_NODE0)
            print(f'    [Spin -> node 0 heading] {s2_dt:.1f}s | '
                  f'delta={math.degrees(s2_delta):.1f}deg | {"OK" if s2_ok else "FAIL"}')

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)

            fwd_rr = fwd_data.get('replanning', {}).get('total_reroutes', 0) if fwd_data else 0
            ret_rr = ret_data.get('replanning', {}).get('total_reroutes', 0) if ret_data else 0
            print(f'  Run {run}: {"OK" if ok else "FAIL"} | reroutes: fwd={fwd_rr} ret={ret_rr}')

            consec_fail = consec_fail + 1 if not ok else 0
            if consec_fail >= MAX_CONSECUTIVE_FAILS:
                print(f'\n  {MAX_CONSECUTIVE_FAILS} consecutive failures, aborting')
                break

        p = sum(results)
        total_reroutes = 0
        for f in self.sdir.glob('test3_*.json'):
            with open(f) as fh:
                d = json.load(fh)
                total_reroutes += d.get('replanning', {}).get('total_reroutes', 0)
        print(f'\n  Test 3: {p}/{len(results)} ok | Total reroutes: {total_reroutes}')
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

        for tname, prefix in [('baseline', 'test2_'), ('adaptive', 'test3_')]:
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
            fe = [r['execution']['xy_error_m'] for r in ok_fwd
                  if r['execution'].get('xy_error_m') is not None]
            rt = [r['execution']['duration_s'] for r in ok_ret]
            rd_l = [r['execution']['distance_traveled_m'] for r in ok_ret]

            ts_data = {
                'total_runs': len(fwd),
                'validated_success_fwd': f'{len(ok_fwd)}/{len(fwd)}',
                'validated_success_ret': f'{len(ok_ret)}/{len(ret)}' if ret else '0/0',
                'forward': {'time_s': stats(ft), 'dist_m': stats(fd), 'error_m': stats(fe)},
                'return': {'time_s': stats(rt), 'dist_m': stats(rd_l)},
            }

            nav_fwd = [r.get('timing', {}).get('navigation_duration_s',
                        r['execution']['duration_s']) for r in ok_fwd]
            nav_ret = [r.get('timing', {}).get('navigation_duration_s',
                        r['execution']['duration_s']) for r in ok_ret]
            prescan_fwd = [r.get('timing', {}).get('prescan_duration_s', 0) for r in ok_fwd]
            ts_data['navigation_only'] = {
                'fwd_time_s': stats(nav_fwd),
                'ret_time_s': stats(nav_ret),
                'prescan_time_s': stats(prescan_fwd) if any(p > 0 for p in prescan_fwd) else {},
            }

            if prefix == 'test3_':
                reroutes_fwd = [r.get('replanning', {}).get('total_reroutes', 0) for r in fwd]
                reroutes_ret = [r.get('replanning', {}).get('total_reroutes', 0) for r in ret]
                ts_data['reroutes_fwd'] = stats(reroutes_fwd)
                ts_data['reroutes_ret'] = stats(reroutes_ret)
                ts_data['total_reroutes'] = sum(reroutes_fwd) + sum(reroutes_ret)
                ts_data['runs_with_reroute'] = sum(1 for r in reroutes_fwd + reroutes_ret if r > 0)

                plan_times = []
                for r in fwd + ret:
                    for seg in r.get('replanning', {}).get('segments', []):
                        plan_times.append(seg.get('plan_ms', 0))
                if plan_times:
                    ts_data['planning_ms'] = stats(plan_times)

                route_types = []
                routes_changed = 0
                for r in fwd + ret:
                    ra = r.get('route_analysis', {})
                    rt_type = ra.get('initial_route_type', '?')
                    if rt_type != '?':
                        route_types.append(rt_type)
                    if ra.get('route_changed', False):
                        routes_changed += 1
                ts_data['route_analysis'] = {
                    'route_types': route_types,
                    'routes_changed': routes_changed,
                    'total_legs': len(fwd) + len(ret),
                }

            mod_all = [r['min_object_distance']['min_distance_m']
                       for r in ok_fwd + ok_ret
                       if r.get('min_object_distance', {}).get('min_distance_m') is not None]
            if mod_all:
                ts_data['min_object_distance_m'] = stats(mod_all)

            all_runs = ok_fwd + ok_ret
            cpu_means = [r['system_metrics']['cpu']['mean']
                         for r in all_runs if r.get('system_metrics', {}).get('cpu')]
            mem_means = [r['system_metrics']['memory']['mean']
                         for r in all_runs if r.get('system_metrics', {}).get('memory')]
            temp_maxs = [r['system_metrics']['cpu_temp']['max']
                         for r in all_runs if r.get('system_metrics', {}).get('cpu_temp')]
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

        if 'baseline' in summary['tests'] and 'adaptive' in summary['tests']:
            b = summary['tests']['baseline']
            a = summary['tests']['adaptive']
            b_nav = b.get('navigation_only', {}).get('fwd_time_s', {})
            a_nav = a.get('navigation_only', {}).get('fwd_time_s', {})
            b_mod = b.get('min_object_distance_m', {})
            a_mod = a.get('min_object_distance_m', {})
            summary['comparison'] = {
                'nav_time_fwd_s': {
                    'baseline': b_nav.get('mean', 0),
                    'adaptive': a_nav.get('mean', 0),
                },
                'min_obj_dist_m': {
                    'baseline': b_mod.get('mean', 0),
                    'adaptive': a_mod.get('mean', 0),
                },
                'success_rate': {
                    'baseline': b.get('validated_success_fwd', '?'),
                    'adaptive': a.get('validated_success_fwd', '?'),
                },
            }

        self._save('summary.json', summary)
        for tn, td in summary.get('tests', {}).items():
            print(f'  {tn}: fwd={td["validated_success_fwd"]} ret={td["validated_success_ret"]}')
            nav = td.get('navigation_only', {}).get('fwd_time_s', {})
            d = td.get('forward', {}).get('dist_m', {})
            if nav:
                print(f'    Fwd nav: {nav.get("mean",0):.1f}+-{nav.get("std",0):.1f}s | '
                      f'{d.get("mean",0):.2f}+-{d.get("std",0):.2f}m')
            mod = td.get('min_object_distance_m', {})
            if mod:
                print(f'    Min obj dist: {mod.get("mean",0):.2f}+-{mod.get("std",0):.2f}m')

        if 'comparison' in summary:
            c = summary['comparison']
            print(f'  --- Comparison ---')
            print(f'    Nav time fwd: baseline={c["nav_time_fwd_s"]["baseline"]:.1f}s '
                  f'adaptive={c["nav_time_fwd_s"]["adaptive"]:.1f}s')
            print(f'    Min obj dist: baseline={c["min_obj_dist_m"]["baseline"]:.2f}m '
                  f'adaptive={c["min_obj_dist_m"]["adaptive"]:.2f}m')


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='SAIM Xplorer Experimental Suite v4')
    parser.add_argument('--test', type=str, default='1',
                        choices=['0', '1', '2', '3', 'all'])
    parser.add_argument('--runs', type=int, default=5,
                        help='Numar runs pentru ambele teste (folosit daca nu specifici --runs-baseline sau --runs-adaptive)')
    parser.add_argument('--runs-baseline', type=int, default=None,
                        help='Numar runs Test 2 (baseline MPPI). Suprascrie --runs.')
    parser.add_argument('--runs-adaptive', type=int, default=None,
                        help='Numar runs Test 3 (adaptive semantic). Suprascrie --runs.')
    parser.add_argument('--session', type=str, default=None)
    parser.add_argument('--wait-secs', type=int, default=60,
                        help='Seconds to wait for semantic repopulation in setup')
    args = parser.parse_args()

    # Determina runs pentru fiecare test
    runs_baseline = args.runs_baseline if args.runs_baseline is not None else args.runs
    runs_adaptive = args.runs_adaptive if args.runs_adaptive is not None else args.runs

    sdir = EXPERIMENTS_DIR / (args.session or f'session_{ts_str()}')
    rclpy.init()
    tester = ExperimentalTester(sdir)

    hdr('SAIM Xplorer, Experimental Suite v4')
    print(f'  Session: {sdir}')
    print(f'  Runs baseline: {runs_baseline} | Runs adaptive: {runs_adaptive} | Test: {args.test}')
    print(f'  CPU monitoring: {"psutil OK" if HAS_PSUTIL else "not installed"}')
    print(f'  Target yaw node0: {math.degrees(TARGET_YAW_NODE0):.2f}deg')
    print(f'  Target yaw node9: {math.degrees(TARGET_YAW_NODE9):.2f}deg')

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
            res['test2'] = tester.run_test2(runs_baseline)
        if args.test in ('3', 'all'):
            res['test3'] = tester.run_test3(runs_adaptive)
        if args.test in ('2', '3', 'all'):
            tester.gen_summary()
    except KeyboardInterrupt:
        print('\n  Interrupted, saving...')
        tester.gen_summary()

    hdr('FINAL REPORT')
    for n, p in res.items():
        print(f'  {n}: {"OK" if p else "FAIL"}')
    print(f'  Data: {sdir}')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()