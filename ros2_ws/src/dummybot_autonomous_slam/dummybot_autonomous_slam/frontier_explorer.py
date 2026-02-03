#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import numpy as np
import math
from collections import deque


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Parametri
        self.declare_parameter('frontier_min_size', 10)  # Mărimea minimă a frontierei
        self.declare_parameter('exploration_radius', 3.0)  # Raza de căutare frontiere
        self.declare_parameter('goal_tolerance', 0.3)  # Toleranță ajungere la goal
        self.declare_parameter('min_frontier_distance', 0.5)  # Distanță minimă între frontiere
        self.declare_parameter('robot_radius', 0.25)  # Raza robotului pentru verificări
        self.declare_parameter('max_exploration_time', 600.0)  # Timp maxim explorare (secunde)
        
        self.frontier_min_size = self.get_parameter('frontier_min_size').value
        self.exploration_radius = self.get_parameter('exploration_radius').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.min_frontier_distance = self.get_parameter('min_frontier_distance').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.max_exploration_time = self.get_parameter('max_exploration_time').value
        
        # State
        self.map_data = None
        self.map_info = None
        self.current_goal = None
        self.exploration_active = True
        self.start_time = self.get_clock().now()
        
        # QoS pentru map (trebuie transient local pentru map_server)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        # Publishers pentru vizualizare
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_markers',
            10
        )
        
        # Action client pentru navigație
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer pentru explorare
        self.exploration_timer = self.create_timer(2.0, self.exploration_cycle)
        
        self.get_logger().info('🤖 DummyBot Autonomous SLAM Explorer started!')
        self.get_logger().info(f'📊 Parameters: min_size={self.frontier_min_size}, radius={self.exploration_radius}m')

    def map_callback(self, msg):
        """Callback pentru primirea hărții"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def exploration_cycle(self):
        """Ciclu principal de explorare"""
        if not self.exploration_active:
            return
            
        # Verifică timeout
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.max_exploration_time:
            self.get_logger().info('⏱️ Maximum exploration time reached!')
            self.exploration_active = False
            return
        
        if self.map_data is None:
            self.get_logger().warn('⚠️ Waiting for map data...')
            return
        
        # Dacă avem un goal activ, așteaptă
        if self.current_goal is not None:
            return
        
        # Găsește frontiere
        frontiers = self.find_frontiers()
        
        if len(frontiers) == 0:
            self.get_logger().info('✅ No more frontiers found! Exploration complete!')
            self.exploration_active = False
            return
        
        # Vizualizează frontierele
        self.visualize_frontiers(frontiers)
        
        # Alege cea mai bună frontieră
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier is None:
            self.get_logger().warn('⚠️ No valid frontier found, waiting...')
            return
        
        # Trimite goal
        self.send_navigation_goal(best_frontier)

    def find_frontiers(self):
        """Găsește toate frontierele pe hartă"""
        frontiers = []
        
        height, width = self.map_data.shape
        visited = np.zeros_like(self.map_data, dtype=bool)
        
        # Caută celule de frontieră (libere adiacente cu necunoscute)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if visited[y, x]:
                    continue
                
                # Verifică dacă e celulă de frontieră
                if self.is_frontier_cell(x, y):
                    # Explorează frontiera folosind BFS
                    frontier = self.explore_frontier(x, y, visited)
                    
                    if len(frontier) >= self.frontier_min_size:
                        # Calculează centroidul frontierei
                        centroid = self.calculate_centroid(frontier)
                        frontiers.append({
                            'cells': frontier,
                            'centroid': centroid,
                            'size': len(frontier)
                        })
        
        self.get_logger().info(f'🎯 Found {len(frontiers)} frontiers')
        return frontiers

    def is_frontier_cell(self, x, y):
        """Verifică dacă o celulă este parte dintr-o frontieră"""
        # Celula trebuie să fie liberă
        if self.map_data[y, x] != 0:
            return False
        
        # Verifică vecinii pentru celule necunoscute (-1)
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                ny, nx = y + dy, x + dx
                if 0 <= ny < self.map_data.shape[0] and 0 <= nx < self.map_data.shape[1]:
                    if self.map_data[ny, nx] == -1:
                        return True
        
        return False

    def explore_frontier(self, start_x, start_y, visited):
        """Explorează o frontieră folosind BFS"""
        frontier_cells = []
        queue = deque([(start_x, start_y)])
        visited[start_y, start_x] = True
        
        while queue:
            x, y = queue.popleft()
            frontier_cells.append((x, y))
            
            # Verifică vecinii
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    
                    nx, ny = x + dx, y + dy
                    
                    if (0 <= ny < self.map_data.shape[0] and 
                        0 <= nx < self.map_data.shape[1] and
                        not visited[ny, nx] and
                        self.is_frontier_cell(nx, ny)):
                        
                        visited[ny, nx] = True
                        queue.append((nx, ny))
        
        return frontier_cells

    def calculate_centroid(self, cells):
        """Calculează centroidul unei liste de celule"""
        if not cells:
            return None
        
        x_sum = sum(cell[0] for cell in cells)
        y_sum = sum(cell[1] for cell in cells)
        
        centroid_x = x_sum / len(cells)
        centroid_y = y_sum / len(cells)
        
        # Convertește din coordonate grid în coordonate world
        world_x = centroid_x * self.map_info.resolution + self.map_info.origin.position.x
        world_y = centroid_y * self.map_info.resolution + self.map_info.origin.position.y
        
        return (world_x, world_y)

    def select_best_frontier(self, frontiers):
        """Selectează cea mai bună frontieră pe baza distanței și dimensiunii"""
        if not frontiers:
            return None
        
        # Pozitia robotului este la origine (0, 0) în frame-ul map
        robot_pos = (0.0, 0.0)
        
        best_frontier = None
        best_score = -float('inf')
        
        for frontier in frontiers:
            centroid = frontier['centroid']
            
            # Calculează distanța
            distance = math.sqrt(
                (centroid[0] - robot_pos[0])**2 + 
                (centroid[1] - robot_pos[1])**2
            )
            
            # Skip dacă e prea departe
            if distance > self.exploration_radius:
                continue
            
            # Scor: prioritizează frontierele mari și apropiate
            # Score = size / distance (mai mare = mai bun)
            score = frontier['size'] / (distance + 0.1)  # +0.1 pentru evitare diviziune cu 0
            
            if score > best_score:
                best_score = score
                best_frontier = frontier
        
        return best_frontier

    def send_navigation_goal(self, frontier):
        """Trimite un goal de navigație către frontieră"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Navigation action server not available!')
            return
        
        centroid = frontier['centroid']
        
        # Creează PoseStamped pentru goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = centroid[0]
        goal_pose.pose.position.y = centroid[1]
        goal_pose.pose.position.z = 0.0
        
        # Orientare implicită (nu contează pentru explorare)
        goal_pose.pose.orientation.w = 1.0
        
        # Creează goal pentru action
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f'🎯 Sending goal to frontier at ({centroid[0]:.2f}, {centroid[1]:.2f})')
        self.get_logger().info(f'   Frontier size: {frontier["size"]} cells')
        
        self.current_goal = centroid
        
        # Trimite goal
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback pentru răspunsul la trimiterea goal-ului"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ Goal rejected!')
            self.current_goal = None
            return
        
        self.get_logger().info('✓ Goal accepted!')
        
        # Așteaptă rezultatul
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback(self, feedback_msg):
        """Callback pentru feedback navigație"""
        pass  # Poți adăuga logging aici dacă vrei

    def navigation_result_callback(self, future):
        """Callback pentru rezultatul navigației"""
        result = future.result().result
        status = future.result().status
        
        self.current_goal = None
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Goal reached successfully!')
        else:
            self.get_logger().warn(f'⚠️ Goal failed with status: {status}')

    def visualize_frontiers(self, frontiers):
        """Publică markere pentru vizualizarea frontierelor în RViz"""
        marker_array = MarkerArray()
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            centroid = frontier['centroid']
            marker.pose.position.x = centroid[0]
            marker.pose.position.y = centroid[1]
            marker.pose.position.z = 0.2
            
            marker.pose.orientation.w = 1.0
            
            # Mărimea proporțională cu dimensiunea frontierei
            size = min(0.3, 0.1 + frontier['size'] * 0.001)
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            
            # Culoare verde pentru frontiere
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime.sec = 2
            
            marker_array.markers.append(marker)
        
        self.frontier_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Autonomous exploration stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()