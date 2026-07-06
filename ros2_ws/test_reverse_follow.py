#!/usr/bin/env python3
"""
Test reverse navigation: goal(9) → start(0) pe graful DummyBot (10 noduri).
Verifică navigație bidirecțională cu penalty-uri semantice.
"""

import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeRoute, FollowPath

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

def identify_route(path_poses):
    if not path_poses:
        return 'UNKNOWN'
    altA = GRAPH_NODES[6]
    altB = GRAPH_NODES[5]
    min_dist_A = float('inf')
    min_dist_B = float('inf')
    for p in path_poses:
        px = p.pose.position.x
        py = p.pose.position.y
        dA = math.sqrt((px - altA['x'])**2 + (py - altA['y'])**2)
        dB = math.sqrt((px - altB['x'])**2 + (py - altB['y'])**2)
        min_dist_A = min(min_dist_A, dA)
        min_dist_B = min(min_dist_B, dB)
    return 'A' if min_dist_A < min_dist_B else 'B'

rclpy.init()
node = Node('test_reverse_follow')
route_client = ActionClient(node, ComputeRoute, '/compute_route')
follow_client = ActionClient(node, FollowPath, '/follow_path')
route_client.wait_for_server(5.0)
follow_client.wait_for_server(5.0)
print('Servere OK')

# ComputeRoute: node 9 (goal) → node 0 (start)
goal = ComputeRoute.Goal()
goal.use_start = True
goal.use_poses = True

goal.start = PoseStamped()
goal.start.header.frame_id = 'map'
goal.start.pose.position.x = GRAPH_NODES[9]['x']
goal.start.pose.position.y = GRAPH_NODES[9]['y']
goal.start.pose.orientation.w = 1.0

goal.goal = PoseStamped()
goal.goal.header.frame_id = 'map'
goal.goal.pose.position.x = GRAPH_NODES[0]['x']
goal.goal.pose.position.y = GRAPH_NODES[0]['y']
goal.goal.pose.orientation.w = 1.0

t0 = time.time()
future = route_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
gh = future.result()
result_future = gh.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
r = result_future.result().result
dt = time.time() - t0

n_poses = len(r.path.poses)
nodes_ids = [n.nodeid for n in r.route.nodes]
edges_ids = [e.edgeid for e in r.route.edges]
route_choice = identify_route(r.path.poses)

print(f'ComputeRoute (reverse): {n_poses} pts, {dt*1000:.0f}ms, cost={r.route.route_cost:.2f}')
print(f'Ruta: nodes={nodes_ids}, edges={edges_ids}')
print(f'Calea aleasă: {"B (altB/node5)" if route_choice == "B" else "A (altA/node6)"}')

# FollowPath
print('\nFollowPath (reverse) — ROBOTUL SE MIȘCĂ!')
time.sleep(2.0)
fp_goal = FollowPath.Goal()
fp_goal.path = r.path
t0 = time.time()
future = follow_client.send_goal_async(fp_goal)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
gh2 = future.result()
result_future2 = gh2.get_result_async()
while not result_future2.done():
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f'  Execuție: {time.time()-t0:.1f}s...', end='\r', flush=True)
dt = time.time() - t0
print(f'\nFollowPath complet (reverse): {dt:.2f}s')
print('DONE')

node.destroy_node()
rclpy.shutdown()