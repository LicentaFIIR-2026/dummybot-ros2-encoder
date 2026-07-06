#!/usr/bin/env python3
"""
SAIM Xplorer — Semantic Navigation Experimental Test Suite v2
=============================================================

Protocol experimental complet pentru articol Q1:

  Setup (--test 0):
      Resetează semantic_objects.geojson, așteaptă repopulare,
      forțează regenerare graf semantic + nav2, snapshot condiții.

  Test 1 (--test 1): ComputeRoute — doar planificare
      Verifică alegerea traseului pe baza penalty-urilor.

  Test 2 (--test 2): ComputeRoute + FollowPath — navigație semantică
      Forward (start→goal) + Spin 180° + Return (goal→start) + Spin 180°
      Repetă N ori (--runs N).

  Test 3 (--test 3): NavigateToPose — baseline fără semantică
      Forward + Spin + Return + Spin, repetă N ori.

  All (--test all): Setup + Test1 + Test2 + Test3

Output: experiments/session_TIMESTAMP/ cu JSON per run + summary.

Utilizare:
  python3 test_semantic_navigation_v2.py --test 0          # doar setup
  python3 test_semantic_navigation_v2.py --test 2 --runs 5 # 5 run-uri semantic
  python3 test_semantic_navigation_v2.py --test all --runs 5

Cerințe:
  - Nav2 stack activ (AMCL, controller_server, planner_server, behavior_server)
  - Route Server activ
  - semantic_localizer activ
  - Robotul localizat pe hartă

Graf: 10 noduri, 28 edges (v2 FIIR)
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

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose, Spin
from nav2_msgs.srv import SetRouteGraph

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


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

# Paths on RPi5
SEMANTIC_OBJECTS_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/semantic_objects.geojson')
ROUTE_GRAPH_BASE_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_fiir.geojson')
ROUTE_GRAPH_SEMANTIC_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_fiir_semantic.geojson')
ROUTE_GRAPH_NAV2_PATH = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/maps/route_graph_fiir_nav2.geojson')
EXPERIMENTS_DIR = Path('/home/saim/dummybot-ros2-encoder/ros2_ws/experiments')

ODOM_TOPIC = '/platform/odom/filtered'
ODOM_SAMPLE_HZ = 2.0

VALID_CLASSES = {
    'person', 'chair', 'bench', 'bottle', 'cup', 'backpack',
    'suitcase', 'potted plant', 'tv', 'laptop', 'book',
}


# =============================================================================
# UTILITIES
# =============================================================================

def make_pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def node_pose(node_id: int, yaw: float = 0.0) -> PoseStamped:
    n = GRAPH_NODES[node_id]
    return make_pose(n['x'], n['y'], yaw)


def yaw_from_quat(oz: float, ow: float) -> float:
    return 2.0 * math.atan2(oz, ow)


def print_header(title: str):
    print('\n' + '=' * 70)
    print(f'  {title}')
    print('=' * 70)


def print_sub(title: str):
    print(f'\n  --- {title} ---')


def ts_str() -> str:
    return datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def identify_route(path_poses) -> Tuple[str, Dict]:
    if not path_poses:
        return 'UNKNOWN', {}

    key_nodes = {
        'in1': GRAPH_NODES[1], 'in2': GRAPH_NODES[2],
        'int2': GRAPH_NODES[3], 'int3': GRAPH_NODES[4],
        'altB': GRAPH_NODES[5], 'altA': GRAPH_NODES[6],
        'int4': GRAPH_NODES[7], 'goal_old': GRAPH_NODES[8],
    }

    min_dists = {name: float('inf') for name in key_nodes}
    for p in path_poses:
        px, py = p.pose.position.x, p.pose.position.y
        for name, node in key_nodes.items():
            d = math.sqrt((px - node['x'])**2 + (py - node['y'])**2)
            min_dists[name] = min(min_dists[name], d)

    THRESH = 0.5
    visited = {name for name, d in min_dists.items() if d < THRESH}

    goes_top = 'altA' in visited or 'goal_old' in visited
    goes_bot = 'altB' in visited or 'int3' in visited

    if goes_top and not goes_bot:
        route_type = 'SUS'
    elif goes_bot and not goes_top:
        route_type = 'JOS'
    elif goes_top and goes_bot:
        route_type = 'MIX'
    else:
        route_type = 'DIRECT'

    return route_type, {
        'visited': sorted(visited),
        'min_dists': {k: round(v, 3) for k, v in min_dists.items()},
    }


def load_semantic_snapshot() -> Dict:
    if not SEMANTIC_OBJECTS_PATH.exists():
        return {'objects_count': 0, 'objects_by_class': {}, 'valid_count': 0}
    try:
        with open(SEMANTIC_OBJECTS_PATH, 'r') as f:
            data = json.load(f)
        by_class = {}
        valid_count = 0
        for feat in data.get('features', []):
            cls = feat.get('properties', {}).get('class_name', 'unknown')
            by_class[cls] = by_class.get(cls, 0) + 1
            if cls in VALID_CLASSES:
                valid_count += 1
        return {
            'objects_count': len(data.get('features', [])),
            'objects_by_class': by_class,
            'valid_count': valid_count,
            'timestamp': data.get('metadata', {}).get('last_updated', ''),
        }
    except Exception as e:
        return {'objects_count': 0, 'error': str(e)}


def load_nav2_penalties() -> Dict[str, float]:
    if not ROUTE_GRAPH_NAV2_PATH.exists():
        return {}
    try:
        with open(ROUTE_GRAPH_NAV2_PATH, 'r') as f:
            data = json.load(f)
        penalties = {}
        for feat in data.get('features', []):
            props = feat.get('properties', {})
            eid = props.get('id')
            meta = props.get('metadata', {})
            if 'penalty' in meta and eid is not None:
                penalties[str(eid)] = meta['penalty']
        return penalties
    except Exception:
        return {}


# =============================================================================
# ODOMETRY TRACKER
# =============================================================================

class OdomTracker:
    def __init__(self, node: Node):
        self._points: List[List[float]] = []
        self._total_dist: float = 0.0
        self._last_x: Optional[float] = None
        self._last_y: Optional[float] = None
        self._last_sample: float = 0.0
        self._interval: float = 1.0 / ODOM_SAMPLE_HZ
        self._velocities: List[float] = []
        self._final_pose: Optional[Tuple[float, float, float]] = None
        self._active: bool = False
        self._lock = threading.Lock()

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE, depth=1)
        node.create_subscription(Odometry, ODOM_TOPIC, self._cb, qos)

    def start(self):
        with self._lock:
            self._points, self._velocities = [], []
            self._total_dist = 0.0
            self._last_x = self._last_y = None
            self._last_sample = 0.0
            self._final_pose = None
            self._active = True

    def stop(self):
        with self._lock:
            self._active = False

    def get_results(self) -> Dict:
        with self._lock:
            avg_v = sum(self._velocities) / len(self._velocities) if self._velocities else 0.0
            max_v = max(self._velocities) if self._velocities else 0.0
            return {
                'distance_traveled_m': round(self._total_dist, 3),
                'avg_velocity_ms': round(avg_v, 3),
                'max_velocity_ms': round(max_v, 3),
                'final_pose': self._final_pose,
                'trajectory_points': len(self._points),
            }

    def get_trajectory(self) -> List[List[float]]:
        with self._lock:
            return list(self._points)

    def _cb(self, msg: Odometry):
        with self._lock:
            if not self._active:
                return
            now = time.monotonic()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = yaw_from_quat(msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w)
            vx = msg.twist.twist.linear.x

            if self._last_x is not None:
                dx = x - self._last_x
                dy = y - self._last_y
                self._total_dist += math.sqrt(dx * dx + dy * dy)
            self._last_x, self._last_y = x, y
            self._velocities.append(abs(vx))

            if now - self._last_sample >= self._interval:
                self._points.append([round(x, 4), round(y, 4), round(now, 3)])
                self._last_sample = now

            self._final_pose = (round(x, 4), round(y, 4), round(yaw, 4))


# =============================================================================
# EXPERIMENTAL TESTER
# =============================================================================

class ExperimentalTester(Node):
    def __init__(self, session_dir: Path):
        super().__init__('experimental_tester')
        self.session_dir = session_dir
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self._cr = ActionClient(self, ComputeRoute, '/compute_route')
        self._fp = ActionClient(self, FollowPath, '/follow_path')
        self._ntp = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._spin_ac = ActionClient(self, Spin, '/spin')
        self._srg = self.create_client(SetRouteGraph, '/route_server/set_route_graph')

        self.odom = OdomTracker(self)
        self._run_ctr = 0

    def wait_for_servers(self, timeout=15.0) -> bool:
        for name, c in [('ComputeRoute', self._cr), ('FollowPath', self._fp),
                        ('NavigateToPose', self._ntp), ('Spin', self._spin_ac)]:
            print(f'  Aștept {name}...', end=' ', flush=True)
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
            print(f'       {time.time()-t0:.1f}s...', end='\r', flush=True)
        print(' ' * 30, end='\r')
        return rf.result()

    def compute_route(self, start, goal):
        g = ComputeRoute.Goal()
        g.start, g.goal = start, goal
        g.use_start = g.use_poses = True
        result = self._send_action(self._cr, g, 30.0)
        if result is None:
            return None, None
        return result.result.path, result.result

    def exec_follow_path(self, path, timeout=180.0) -> bool:
        g = FollowPath.Goal()
        g.path = path
        return self._send_action(self._fp, g, timeout) is not None

    def exec_nav_to_pose(self, goal, timeout=180.0) -> bool:
        g = NavigateToPose.Goal()
        g.pose = goal
        return self._send_action(self._ntp, g, timeout) is not None

    def exec_spin(self, angle=math.pi, timeout=30.0) -> Tuple[bool, float]:
        g = Spin.Goal()
        g.target_yaw = angle
        t0 = time.time()
        result = self._send_action(self._spin_ac, g, timeout)
        return result is not None, time.time() - t0

    def reload_route_graph(self) -> bool:
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

    # ─── Logging helpers ──────────────────────────────────────────

    def _next_run(self) -> int:
        self._run_ctr += 1
        return self._run_ctr

    def _save(self, filename: str, data: Dict):
        p = self.session_dir / filename
        with open(p, 'w') as f:
            json.dump(data, f, indent=2, default=str)
        print(f'  📄 {p.name}')

    def _pos_error(self, goal_pose) -> Dict:
        r = self.odom.get_results()
        fp = r.get('final_pose')
        if fp is None:
            return {'xy_error_m': None, 'yaw_error_rad': None}
        gx = goal_pose.pose.position.x
        gy = goal_pose.pose.position.y
        g_yaw = yaw_from_quat(goal_pose.pose.orientation.z,
                               goal_pose.pose.orientation.w)
        xy = math.sqrt((fp[0] - gx)**2 + (fp[1] - gy)**2)
        yaw_e = abs(fp[2] - g_yaw)
        if yaw_e > math.pi:
            yaw_e = 2 * math.pi - yaw_e
        return {'xy_error_m': round(xy, 4), 'yaw_error_rad': round(yaw_e, 4)}

    def _route_info(self, result) -> Dict:
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

    # =================================================================
    # TEST 0: SESSION SETUP
    # =================================================================

    def run_setup(self):
        print_header('TEST 0: SESSION SETUP')

        # 1. Backup + clear
        print_sub('Pas 1: Reset semantic_objects')
        if SEMANTIC_OBJECTS_PATH.exists():
            bk = self.session_dir / f'semantic_objects_BACKUP_{ts_str()}.geojson'
            shutil.copy2(SEMANTIC_OBJECTS_PATH, bk)
            print(f'  Backup: {bk.name}')
            empty = {'type': 'FeatureCollection',
                     'metadata': {'description': 'Reset', 'frame': 'map',
                                  'generated_by': 'experiment_setup',
                                  'last_updated': iso_now(), 'object_count': 0},
                     'features': []}
            with open(SEMANTIC_OBJECTS_PATH, 'w') as f:
                json.dump(empty, f, indent=2)
            print('  Resetat (0 obiecte)')

        # 2. Wait for repopulation
        print_sub('Pas 2: Aștept repopulare (60s)')
        for i in range(60, 0, -1):
            rclpy.spin_once(self, timeout_sec=1.0)
            if i % 10 == 0:
                s = load_semantic_snapshot()
                print(f'  {i}s — {s["objects_count"]} obiecte ({s.get("valid_count",0)} valide)')

        # 3. Snapshot
        print_sub('Pas 3: Snapshot')
        snap = load_semantic_snapshot()
        print(f'  Total: {snap["objects_count"]} | Valide: {snap.get("valid_count",0)}')
        print(f'  Clase: {snap.get("objects_by_class", {})}')
        if SEMANTIC_OBJECTS_PATH.exists():
            shutil.copy2(SEMANTIC_OBJECTS_PATH,
                        self.session_dir / 'semantic_snapshot_start.geojson')

        # 4. Wait for save cycle
        print_sub('Pas 4: Aștept regenerare graf')
        for i in range(35):
            rclpy.spin_once(self, timeout_sec=1.0)
            if ROUTE_GRAPH_SEMANTIC_PATH.exists():
                age = time.time() - os.path.getmtime(ROUTE_GRAPH_SEMANTIC_PATH)
                if age < 35:
                    print(f'  Graf semantic actualizat (age={age:.0f}s)')
                    break

        # 5. Reload
        print_sub('Pas 5: Reload Route Server')
        ok = self.reload_route_graph()
        print(f'  {"✓" if ok else "⚠ EȘUAT"}')

        # 6. Config
        cfg = {
            'session_id': self.session_dir.name, 'timestamp': iso_now(),
            'graph': {'nodes': len(GRAPH_NODES),
                      'coords': {str(k): v for k, v in GRAPH_NODES.items()}},
            'semantic': snap,
            'penalties': load_nav2_penalties(),
            'params': {'odom_topic': ODOM_TOPIC, 'sample_hz': ODOM_SAMPLE_HZ,
                       'valid_classes': sorted(VALID_CLASSES)},
        }
        self._save('session_config.json', cfg)
        print('\n  ✓ Setup complet!')
        return True

    # =================================================================
    # TEST 1: COMPUTE ROUTE
    # =================================================================

    def run_test1(self):
        print_header('TEST 1: ComputeRoute')
        start, goal = node_pose(0), node_pose(9)

        t0 = time.time()
        path, result = self.compute_route(start, goal)
        dt = time.time() - t0

        if path is None:
            print('  [EROARE] None')
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
        print(f'  Noduri: {ri.get("route_nodes", "?")} | Cost: {ri.get("route_cost", "?")}')
        print('  ✓ Test 1 complet')
        return True

    # =================================================================
    # TEST 2: SEMANTIC NAVIGATION
    # =================================================================

    def _run_semantic_leg(self, start_id, goal_id, run_num, direction):
        """Execute one leg: plan + execute + log."""
        start_p = node_pose(start_id)
        goal_p = node_pose(goal_id)

        # Plan
        t0 = time.time()
        path, result = self.compute_route(start_p, goal_p)
        plan_dt = time.time() - t0

        if path is None or not path.poses:
            print(f'    [EROARE] Planificare {direction} eșuată')
            return None, False

        rt, rd = identify_route(path.poses)
        ri = self._route_info(result)
        pen = load_nav2_penalties()
        ep = {str(e): pen.get(str(e), 0.0) for e in ri.get('route_edges', [])}

        print(f'    Plan: {plan_dt*1000:.0f}ms | {rt} | nodes={ri.get("route_nodes","?")}')

        # Execute
        print('    ⚠ ROBOT SE MIȘCĂ')
        time.sleep(1.0)
        self.odom.start()
        t0 = time.time()
        ok = self.exec_follow_path(path)
        exec_dt = time.time() - t0
        self.odom.stop()

        odom_r = self.odom.get_results()
        traj = self.odom.get_trajectory()
        err = self._pos_error(goal_p)

        print(f'    Exec: {exec_dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err["xy_error_m"]}m | {"✓" if ok else "✗"}')

        data = {
            'test_type': f'semantic_{direction}',
            'run_number': run_num,
            'timestamp': iso_now(),
            'planning': {'duration_ms': round(plan_dt * 1000, 2),
                         'points': len(path.poses), 'route_type': rt,
                         **ri, 'penalties_per_edge': ep},
            'execution': {'duration_s': round(exec_dt, 3), 'success': ok,
                          **odom_r, **err},
            'trajectory': traj,
            'semantic': load_semantic_snapshot(),
        }
        return data, ok

    def run_test2(self, n_runs=5):
        print_header(f'TEST 2: Semantic — {n_runs} run-uri')
        results = []

        for run in range(1, n_runs + 1):
            print_sub(f'RUN {run}/{n_runs}')

            # Forward
            print('  [Forward] start(0) → goal(9)')
            fwd_data, fwd_ok = self._run_semantic_leg(0, 9, run, 'forward')
            if fwd_data:
                self._save(f'test2_run{run:02d}_forward.json', fwd_data)

            # Spin
            print('    [Spin 180°]')
            s_ok, s_dt = self.exec_spin(math.pi)
            print(f'    {s_dt:.1f}s | {"✓" if s_ok else "✗"}')

            # Return
            print('  [Return] goal(9) → start(0)')
            ret_data, ret_ok = self._run_semantic_leg(9, 0, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {'duration_s': round(s_dt, 2), 'success': s_ok}
                # Check penalty changes
                if fwd_data and ret_data:
                    fwd_pen = fwd_data.get('planning', {}).get('penalties_per_edge', {})
                    ret_pen = ret_data.get('planning', {}).get('penalties_per_edge', {})
                    ret_data['penalties_changed'] = fwd_pen != ret_pen
                self._save(f'test2_run{run:02d}_return.json', ret_data)

            # Spin back
            print('    [Spin 180° revenire]')
            s2_ok, s2_dt = self.exec_spin(math.pi)
            print(f'    {s2_dt:.1f}s | {"✓" if s2_ok else "✗"}')

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)
            print(f'  Run {run}: {"✓" if ok else "✗"}')

        p = sum(results)
        print(f'\n  Test 2: {p}/{n_runs} reușite')
        return p == n_runs

    # =================================================================
    # TEST 3: BASELINE
    # =================================================================

    def _run_baseline_leg(self, goal_id, run_num, direction):
        goal_p = node_pose(goal_id)

        print('    ⚠ ROBOT SE MIȘCĂ')
        time.sleep(1.0)
        self.odom.start()
        t0 = time.time()
        ok = self.exec_nav_to_pose(goal_p)
        dt = time.time() - t0
        self.odom.stop()

        odom_r = self.odom.get_results()
        traj = self.odom.get_trajectory()
        err = self._pos_error(goal_p)

        print(f'    Exec: {dt:.1f}s | {odom_r["distance_traveled_m"]}m | '
              f'err={err["xy_error_m"]}m | {"✓" if ok else "✗"}')

        data = {
            'test_type': f'baseline_{direction}',
            'run_number': run_num,
            'timestamp': iso_now(),
            'execution': {'duration_s': round(dt, 3), 'success': ok,
                          **odom_r, **err},
            'trajectory': traj,
            'semantic': load_semantic_snapshot(),
        }
        return data, ok

    def run_test3(self, n_runs=5):
        print_header(f'TEST 3: Baseline — {n_runs} run-uri')
        results = []

        for run in range(1, n_runs + 1):
            print_sub(f'RUN {run}/{n_runs}')

            print('  [Forward] → goal(9) via NavigateToPose')
            fwd_data, fwd_ok = self._run_baseline_leg(9, run, 'forward')
            if fwd_data:
                self._save(f'test3_run{run:02d}_forward.json', fwd_data)

            print('    [Spin 180°]')
            s_ok, s_dt = self.exec_spin(math.pi)
            print(f'    {s_dt:.1f}s | {"✓" if s_ok else "✗"}')

            print('  [Return] → start(0) via NavigateToPose')
            ret_data, ret_ok = self._run_baseline_leg(0, run, 'return')
            if ret_data:
                ret_data['spin_before'] = {'duration_s': round(s_dt, 2), 'success': s_ok}
                self._save(f'test3_run{run:02d}_return.json', ret_data)

            print('    [Spin 180° revenire]')
            s2_ok, s2_dt = self.exec_spin(math.pi)
            print(f'    {s2_dt:.1f}s | {"✓" if s2_ok else "✗"}')

            ok = (fwd_ok or False) and (ret_ok or False)
            results.append(ok)
            print(f'  Run {run}: {"✓" if ok else "✗"}')

        p = sum(results)
        print(f'\n  Test 3: {p}/{n_runs} reușite')
        return p == n_runs

    # =================================================================
    # SUMMARY
    # =================================================================

    def gen_summary(self):
        print_sub('Summary')
        import statistics

        def stats(vals):
            if not vals:
                return {}
            return {
                'mean': round(statistics.mean(vals), 3),
                'std': round(statistics.stdev(vals), 3) if len(vals) > 1 else 0,
                'min': round(min(vals), 3),
                'max': round(max(vals), 3), 'n': len(vals)}

        summary = {'session': self.session_dir.name, 'timestamp': iso_now(), 'tests': {}}

        for tname, prefix in [('semantic', 'test2_'), ('baseline', 'test3_')]:
            fwd, ret = [], []
            for f in sorted(self.session_dir.glob(f'{prefix}*_forward.json')):
                with open(f) as fh:
                    fwd.append(json.load(fh))
            for f in sorted(self.session_dir.glob(f'{prefix}*_return.json')):
                with open(f) as fh:
                    ret.append(json.load(fh))
            if not fwd:
                continue

            ft = [r['execution']['duration_s'] for r in fwd if r['execution']['success']]
            fd = [r['execution']['distance_traveled_m'] for r in fwd if r['execution']['success']]
            fe = [r['execution']['xy_error_m'] for r in fwd
                  if r['execution']['success'] and r['execution'].get('xy_error_m') is not None]
            rt = [r['execution']['duration_s'] for r in ret if r['execution']['success']]
            rd = [r['execution']['distance_traveled_m'] for r in ret if r['execution']['success']]

            ts_data = {
                'runs': len(fwd),
                'success': f'{sum(1 for r in fwd if r["execution"]["success"])}/{len(fwd)}',
                'forward': {'time_s': stats(ft), 'dist_m': stats(fd), 'error_m': stats(fe)},
                'return': {'time_s': stats(rt), 'dist_m': stats(rd)},
            }

            if prefix == 'test2_':
                pt = [r['planning']['duration_ms'] for r in fwd if 'planning' in r]
                rts = [r['planning'].get('route_type', '?') for r in fwd if 'planning' in r]
                ts_data['planning_ms'] = stats(pt)
                ts_data['route_types'] = rts
                ts_data['penalties_changed'] = sum(
                    1 for r in ret if r.get('penalties_changed', False))

            summary['tests'][tname] = ts_data

        self._save('summary.json', summary)

        for tn, td in summary.get('tests', {}).items():
            print(f'  {tn}: {td["success"]} ok')
            f = td.get('forward', {}).get('time_s', {})
            d = td.get('forward', {}).get('dist_m', {})
            if f:
                print(f'    Fwd: {f.get("mean",0):.1f}±{f.get("std",0):.1f}s | '
                      f'{d.get("mean",0):.2f}±{d.get("std",0):.2f}m')


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='SAIM Xplorer Experimental Suite v2')
    parser.add_argument('--test', type=str, default='1',
                        choices=['0', '1', '2', '3', 'all'])
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--session', type=str, default=None)
    args = parser.parse_args()

    session = args.session or f'session_{ts_str()}'
    sdir = EXPERIMENTS_DIR / session

    rclpy.init()
    tester = ExperimentalTester(sdir)

    print_header('SAIM Xplorer — Experimental Suite v2')
    print(f'  Sesiune: {sdir}')
    print(f'  Runs: {args.runs} | Test: {args.test}')

    if not tester.wait_for_servers():
        print('\n  [EROARE] Servere indisponibile')
        tester.destroy_node()
        rclpy.shutdown()
        return

    res = {}
    try:
        if args.test in ('0', 'all'):
            res['setup'] = tester.run_setup()
        if args.test in ('1', 'all'):
            res['test1'] = tester.run_test1()
        if args.test in ('2', 'all'):
            res['test2'] = tester.run_test2(args.runs)
        if args.test in ('3', 'all'):
            res['test3'] = tester.run_test3(args.runs)
        if args.test in ('2', '3', 'all'):
            tester.gen_summary()
    except KeyboardInterrupt:
        print('\n  ⚠ Întrerupt — salvez...')
        tester.gen_summary()

    print_header('RAPORT FINAL')
    for n, p in res.items():
        print(f'  {n}: {"✓" if p else "✗"}')
    print(f'  Date: {sdir}')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
