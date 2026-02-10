#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import DynamicEdges
from nav2_msgs.msg import EdgeCost

class TestDynamicEdges(Node):
    def __init__(self):
        super().__init__('test_dynamic_edges')
        
        # Client pentru service
        self.client = self.create_client(
            DynamicEdges,
            '/route_server/DynamicEdgesScorer/adjust_edges'
        )
        
        # Așteaptă service
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        
        self.get_logger().info('Service available! Starting test...')
        
        # Test 1: Cost normal pentru toate muchiile
        self.test_normal_costs()
        
        # Test 2: Cost mare pe muchie 10
        self.test_high_cost_edge_10()
        
        # Test 3: Închide complet muchie 12
        self.test_close_edge_12()

    def test_normal_costs(self):
        """Test 1: Toate muchiile cu cost normal (1.0)"""
        self.get_logger().info('TEST 1: Setting normal costs (1.0) for all edges')
        
        request = DynamicEdges.Request()
        request.adjust_edges = [
            EdgeCost(edgeid=10, cost=1.0),
            EdgeCost(edgeid=11, cost=1.0),
            EdgeCost(edgeid=12, cost=1.0),
            EdgeCost(edgeid=13, cost=1.0),
            EdgeCost(edgeid=14, cost=1.0),
            EdgeCost(edgeid=15, cost=1.0),
        ]
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✓ Test 1 SUCCESS: All edges set to 1.0')
        else:
            self.get_logger().error('✗ Test 1 FAILED')

    def test_high_cost_edge_10(self):
        """Test 2: Cost mare pe muchie 10 (masa-capat → usa-fata)"""
        self.get_logger().info('TEST 2: Setting high cost (10.0) on edge 10')
        
        request = DynamicEdges.Request()
        request.adjust_edges = [
            EdgeCost(edgeid=10, cost=10.0),  # Cost foarte mare
        ]
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✓ Test 2 SUCCESS: Edge 10 cost = 10.0')
            self.get_logger().info('  → Robot ar trebui să evite muchea 10 acum!')
        else:
            self.get_logger().error('✗ Test 2 FAILED')

    def test_close_edge_12(self):
        """Test 3: Închide complet muchie 12 (usa-fata → usa-spate)"""
        self.get_logger().info('TEST 3: Closing edge 12 completely')
        
        request = DynamicEdges.Request()
        request.closed_edges = [12]  # Închide complet muchea
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('✓ Test 3 SUCCESS: Edge 12 closed')
            self.get_logger().info('  → Robot NU poate folosi muchea 12!')
        else:
            self.get_logger().error('✗ Test 3 FAILED')

def main(args=None):
    rclpy.init(args=args)
    node = TestDynamicEdges()
    
    node.get_logger().info('All tests completed!')
    node.get_logger().info('Try navigating with smart_waypoint_nav.py to see effects')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
