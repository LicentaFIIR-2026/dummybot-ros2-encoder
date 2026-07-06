#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import DynamicEdges
from nav2_msgs.msg import EdgeCost

class BlockRouteA(Node):
    def __init__(self):
        super().__init__('block_route_a')
        
        self.client = self.create_client(
            DynamicEdges,
            '/route_server/DynamicEdgesScorer/adjust_edges'
        )
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
        
        # Blochează muchiile int2 → altA → goal (edges 14, 18)
        self.get_logger().info('🚫 BLOCKING Route A (altA)')
        
        request = DynamicEdges.Request()
        request.adjust_edges = [
            EdgeCost(edgeid=14, cost=100.0),  # int2 → altA
            EdgeCost(edgeid=18, cost=100.0),  # altA → goal
        ]
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✓ Route A BLOCKED')
            self.get_logger().info('  Robot should now use Route B (altB)!')
            rclpy.spin(self)

def main():
    rclpy.init()
    node = BlockRouteA()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
