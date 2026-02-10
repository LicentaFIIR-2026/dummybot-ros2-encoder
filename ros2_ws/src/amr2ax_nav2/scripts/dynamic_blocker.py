#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import DynamicEdges
import threading

class DynamicBlocker(Node):
    def __init__(self):
        super().__init__('dynamic_blocker')
        
        self.client = self.create_client(
            DynamicEdges,
            '/route_server/DynamicEdgesScorer/adjust_edges'
        )
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('Waiting for service...')
        
        print('✓ Service connected!')
        
        self.route_a_blocked = False
        self.route_b_blocked = False
        
        self.input_thread = threading.Thread(target=self.interactive_loop, daemon=True)
        self.input_thread.start()
        
        print('\n' + '='*60)
        print('COMENZI DISPONIBILE:')
        print('  a  - Blocheaza/Deblocheaza Ruta A (altA)')
        print('  b  - Blocheaza/Deblocheaza Ruta B (altB)')
        print('  r  - Reset (deblocheaza tot)')
        print('  s  - Status')
        print('  q  - Quit')
        print('='*60 + '\n')
    
    def interactive_loop(self):
        while rclpy.ok():
            try:
                cmd = input('Comanda (a/b/r/s/q): ').strip().lower()
                if cmd == 'q':
                    rclpy.shutdown()
                    break
                elif cmd == 'a':
                    self.toggle_route('a')
                elif cmd == 'b':
                    self.toggle_route('b')
                elif cmd == 'r':
                    self.reset_all()
                elif cmd == 's':
                    self.show_status()
                else:
                    print('Comanda invalida! Foloseste: a, b, r, s, q')
            except EOFError:
                break
    
    def toggle_route(self, route):
        if route == 'a':
            self.route_a_blocked = not self.route_a_blocked
            blocked = self.route_a_blocked
            edges = [14, 15, 18, 19]
            name = 'Ruta A (altA)'
        else:
            self.route_b_blocked = not self.route_b_blocked
            blocked = self.route_b_blocked
            edges = [16, 17, 20, 21]
            name = 'Ruta B (altB)'
        
        request = DynamicEdges.Request()
        if blocked:
            print(f'🚫 BLOCKING {name}')
            request.closed_edges = edges
        else:
            print(f'✅ UNBLOCKING {name}')
            request.opened_edges = edges
        
        self.call_service(request)
    
    def reset_all(self):
        print('🔄 RESET: toate rutele deschise')
        self.route_a_blocked = False
        self.route_b_blocked = False
        
        request = DynamicEdges.Request()
        request.opened_edges = [14, 15, 16, 17, 18, 19, 20, 21]
        
        self.call_service(request)
    
    def show_status(self):
        print('\n' + '='*40)
        print('STATUS:')
        print(f'  Ruta A (altA): {"🚫 BLOCKED" if self.route_a_blocked else "✅ OPEN"}')
        print(f'  Ruta B (altB): {"🚫 BLOCKED" if self.route_b_blocked else "✅ OPEN"}')
        print('='*40 + '\n')
    
    def call_service(self, request):
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                print('✓ Actualizat!')
            else:
                print('✗ Service call failed!')
        except Exception as e:
            print(f'Service error: {e}')


def main():
    rclpy.init()
    node = DynamicBlocker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()