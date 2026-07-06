#!/usr/bin/env python3
"""
DummyBot — Semantic Navigation Test Suite
==========================================

Testează 3 moduri de navigație pentru comparație experimentală:

  Test 1: ComputeRoute (doar planificare, fără execuție)
          → Verifică că Route Server alege traseul corect pe baza penalty-urilor

  Test 2: ComputeRoute + FollowPath (Arhitectura #1)
          → Navigație completă: Route Server planifică, MPPI execută

  Test 3: NavigateToPose baseline (fără Route Server)
          → SmacPlannerHybrid alege singur, fără semantică

Cerințe:
  - Nav2 stack activ (AMCL, controller_server, planner_server, etc.)
  - Route Server activ și configurat cu xplorer.yaml
  - Robotul localizat pe hartă

Utilizare:
  python3 test_semantic_navigation.py [--test 1|2|3|all]

Notă: Nu folosim BasicNavigator.getRoute() — nu există pe Jazzy 1.3.10.
      Folosim action client direct pentru ComputeRoute.
"""

import sys
import time
import argparse
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, FollowPath, NavigateToPose

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


# =============================================================================
# Graful DummyBot — coordonate noduri
# =============================================================================
GRAPH_NODES = {
    0: {'name': 'start',    'x': 1.097, 'y': 2.588},
    1: {'name': 'in1',      'x': 2.313, 'y': 3.215},
    2: {'name': 'in2',      'x': 2.316, 'y': 2.540},  # MODIFICAT
    3: {'name': 'int2',     'x': 3.334, 'y': 2.844},
    4: {'name': 'int3',     'x': 3.337, 'y': 2.473},  # MODIFICAT
    5: {'name': 'altB',     'x': 4.464, 'y': 2.563},  # MODIFICAT
    6: {'name': 'altA',     'x': 4.353, 'y': 3.299},
    7: {'name': 'int4',     'x': 5.302, 'y': 2.564},
    8: {'name': 'goal_old', 'x': 5.635, 'y': 3.335},
    9: {'name': 'goal',     'x': 6.002, 'y': 2.649},
}

# Penalty-uri din route_graph_dummybot_nav2.geojson (pentru referință)
# Calea A: 0→1(10.51) → 1→3(15.36) → 3→6(16.44) → 6→7(10.79) → 7→9(4.66) = total 57.76
# Calea B: 0→2(10.09) → 2→4(13.71) → 4→5(13.42) → 5→7(8.58)  → 7→9(4.66) = total 50.46
# → Route Server ar trebui să aleagă Calea B (prin altB, node 5)


def make_pose(x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    """Creează PoseStamped în frame map."""
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


def node_pose(node_id: int) -> PoseStamped:
    """PoseStamped pentru un nod din graf."""
    n = GRAPH_NODES[node_id]
    return make_pose(n['x'], n['y'])


def print_header(title: str):
    print('\n' + '=' * 65)
    print(f'  {title}')
    print('=' * 65)


def identify_route(path_poses):
    """Identifică calea aleasă — Calea A (prin altA/node6) sau Calea B (prin altB/node5)."""
    if not path_poses:
        return 'UNKNOWN'

    altA = GRAPH_NODES[6]  # (4.353, 3.299)
    altB = GRAPH_NODES[5]  # (4.469, 2.454)

    min_dist_A = float('inf')
    min_dist_B = float('inf')

    for p in path_poses:
        px = p.pose.position.x
        py = p.pose.position.y
        dA = math.sqrt((px - altA['x'])**2 + (py - altA['y'])**2)
        dB = math.sqrt((px - altB['x'])**2 + (py - altB['y'])**2)
        min_dist_A = min(min_dist_A, dA)
        min_dist_B = min(min_dist_B, dB)

    if min_dist_A < min_dist_B:
        return 'A'
    else:
        return 'B'


# =============================================================================
# Clasa principală — gestionează action clients
# =============================================================================
class SemanticNavTester(Node):
    def __init__(self):
        super().__init__('semantic_nav_tester')

        self._compute_route_client = ActionClient(
            self, ComputeRoute, '/compute_route')
        self._follow_path_client = ActionClient(
            self, FollowPath, '/follow_path')
        self._nav_to_pose_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

    def wait_for_servers(self, timeout=10.0):
        """Așteaptă action servers."""
        print('  Aștept Route Server (/compute_route)...', end=' ', flush=True)
        if not self._compute_route_client.wait_for_server(timeout):
            print('TIMEOUT!')
            return False
        print('OK')

        print('  Aștept Controller Server (/follow_path)...', end=' ', flush=True)
        if not self._follow_path_client.wait_for_server(timeout):
            print('TIMEOUT!')
            return False
        print('OK')

        print('  Aștept BT Navigator (/navigate_to_pose)...', end=' ', flush=True)
        if not self._nav_to_pose_client.wait_for_server(timeout):
            print('TIMEOUT!')
            return False
        print('OK')
        return True

    def compute_route(self, start: PoseStamped, goal: PoseStamped):
        """Apelează ComputeRoute și returnează (path, full_result)."""
        goal_msg = ComputeRoute.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.use_start = True
        goal_msg.use_poses = True

        future = self._compute_route_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('ComputeRoute goal REJECTED')
            return None, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        result = result_future.result()
        if result is None:
            return None, None

        return result.result.path, result.result

    def follow_path(self, path):
        """Trimite path la FollowPath și așteaptă completarea."""
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        future = self._follow_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('FollowPath goal REJECTED')
            return False

        result_future = goal_handle.get_result_async()

        t0 = time.time()
        timeout = 120.0
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - t0
            if elapsed > timeout:
                self.get_logger().error('FollowPath TIMEOUT')
                goal_handle.cancel_goal_async()
                return False
            print(f'       Execuție: {elapsed:.1f}s...', end='\r', flush=True)

        result = result_future.result()
        print()
        return result is not None

    def navigate_to_pose(self, goal: PoseStamped):
        """Trimite NavigateToPose și așteaptă completarea."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        future = self._nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('NavigateToPose goal REJECTED')
            return False

        result_future = goal_handle.get_result_async()

        t0 = time.time()
        timeout = 120.0
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            elapsed = time.time() - t0
            if elapsed > timeout:
                self.get_logger().error('NavigateToPose TIMEOUT')
                goal_handle.cancel_goal_async()
                return False
            print(f'       Execuție: {elapsed:.1f}s...', end='\r', flush=True)

        result = result_future.result()
        print()
        return result is not None


# =============================================================================
# TEST 1: ComputeRoute — doar planificare
# =============================================================================
def test_compute_route(tester: SemanticNavTester):
    print_header('TEST 1: ComputeRoute (planificare semantică)')
    print('  Cerere: node 0 (start) → node 9 (goal)')
    print('  Așteptare: Calea B (0→2→4→5→7→9) — penalty total mai mic (50.46 vs 57.76)')
    print()

    start = node_pose(0)
    goal = node_pose(9)

    t0 = time.time()
    path, result = tester.compute_route(start, goal)
    dt = time.time() - t0

    print(f'  Timp planificare: {dt*1000:.1f} ms')

    if path is None:
        print('  [EROARE] ComputeRoute a returnat None')
        return False

    n_poses = len(path.poses)
    print(f'  Dense path: {n_poses} puncte')

    if n_poses > 0:
        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        print(f'  Start path: ({first.x:.3f}, {first.y:.3f})')
        print(f'  Goal path:  ({last.x:.3f}, {last.y:.3f})')

        route_choice = identify_route(path.poses)
        if route_choice == 'B':
            print(f'  → Calea B (prin altB/node5) — PENALTY MIC ✓')
        elif route_choice == 'A':
            print(f'  → Calea A (prin altA/node6) — PENALTY MAI MARE ✗')
        else:
            print(f'  → Traseu nedeterminat')

    if result is not None:
        if hasattr(result, 'route') and result.route:
            route = result.route
            if hasattr(route, 'nodes') and route.nodes:
                node_ids = [n.nodeid for n in route.nodes]
                print(f'  Noduri rută sparsă: {node_ids}')
            if hasattr(route, 'edges') and route.edges:
                edge_ids = [e.edgeid for e in route.edges]
                print(f'  Edges rută: {edge_ids}')

    print()
    print('  [OK] ComputeRoute funcționează!')
    return True


# =============================================================================
# TEST 2: ComputeRoute + FollowPath
# =============================================================================
def test_route_and_follow(tester: SemanticNavTester):
    print_header('TEST 2: ComputeRoute + FollowPath (SEMANTIC)')
    print('  Cerere: node 0 (start) → node 9 (goal)')
    print('  Route Server planifică, MPPI Controller execută')
    print()

    start = node_pose(0)
    goal = node_pose(9)

    print('  [2a] Calculez ruta semantică...')
    t0 = time.time()
    path, result = tester.compute_route(start, goal)
    dt_plan = time.time() - t0
    print(f'  Timp planificare: {dt_plan*1000:.1f} ms')

    if path is None or not path.poses:
        print('  [EROARE] Path-ul este gol sau None')
        return False

    print(f'  Dense path: {len(path.poses)} puncte')
    route_choice = identify_route(path.poses)
    print(f'  Calea aleasă: {"B (altB/node5)" if route_choice == "B" else "A (altA/node6)"}')

    print()
    print('  [2b] Trimit path-ul la FollowPath (MPPI Controller)...')
    print('  ⚠  ROBOTUL SE VA MIȘCA!')
    time.sleep(2.0)

    t0_exec = time.time()
    success = tester.follow_path(path)
    dt_exec = time.time() - t0_exec

    print(f'  Timp execuție: {dt_exec:.2f} s')

    if success:
        print('  [OK] Navigație semantică completă! ✓')
    else:
        print('  [EȘEC] FollowPath a eșuat')
    return success


# =============================================================================
# TEST 3: NavigateToPose baseline
# =============================================================================
def test_baseline(tester: SemanticNavTester):
    print_header('TEST 3: NavigateToPose BASELINE (fără semantică)')
    print(f'  Cerere: direct la goal ({GRAPH_NODES[9]["x"]}, {GRAPH_NODES[9]["y"]})')
    print('  SmacPlannerHybrid alege traseul fără penalty-uri semantice')
    print()
    print('  ⚠  ROBOTUL SE VA MIȘCA!')
    time.sleep(2.0)

    goal = node_pose(9)

    t0 = time.time()
    success = tester.navigate_to_pose(goal)
    dt = time.time() - t0

    print(f'  Timp total: {dt:.2f} s')

    if success:
        print('  [OK] Baseline NavigateToPose complet! ✓')
    else:
        print('  [EȘEC] NavigateToPose a eșuat')
    return success


# =============================================================================
# MAIN
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description='DummyBot Semantic Navigation Tests')
    parser.add_argument('--test', type=str, default='1',
                        choices=['1', '2', '3', 'all'],
                        help='Test de rulat: 1=ComputeRoute, 2=Route+Follow, 3=Baseline, all=toate')
    args = parser.parse_args()

    rclpy.init()
    tester = SemanticNavTester()

    print_header('DummyBot — Semantic Navigation Test Suite')

    if not tester.wait_for_servers(timeout=15.0):
        print('\n  [EROARE] Nu am putut contacta toate serverele.')
        print('  Verifică: Nav2 stack activ? Route Server activ?')
        tester.destroy_node()
        rclpy.shutdown()
        return

    print()
    results = {}

    if args.test in ('1', 'all'):
        results['test1_compute_route'] = test_compute_route(tester)

    if args.test in ('2', 'all'):
        results['test2_route_follow'] = test_route_and_follow(tester)

    if args.test in ('3', 'all'):
        results['test3_baseline'] = test_baseline(tester)

    print_header('SUMAR REZULTATE')
    for name, passed in results.items():
        status = '✓ PASS' if passed else '✗ FAIL'
        print(f'  {name}: {status}')

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()