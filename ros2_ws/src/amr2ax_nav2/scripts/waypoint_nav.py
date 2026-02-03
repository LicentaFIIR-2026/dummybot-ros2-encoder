#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
import yaml
import os
import sys
import math
import threading

class WaypointNavigator(Node):
    def __init__(self, mode='navigate'):
        super().__init__('waypoint_navigator')
        
        self.mode = mode  # 'navigate' sau 'record'
        
        # Path către fișierul de waypoint-uri
        self.waypoints_file = os.path.expanduser(
            '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml'
        )
        
        # Încarcă waypoint-uri
        self.waypoints = self.load_waypoints()
        
        if mode == 'record':
            self.setup_recorder()
        elif mode == 'navigate':
            self.setup_navigator()
    
    # ==================== COMMON FUNCTIONS ====================
    
    def load_waypoints(self):
        """Încarcă waypoint-uri din YAML"""
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        return data['waypoints']
            except Exception as e:
                self.get_logger().warn(f'Eroare încărcare: {e}')
        return {}
    
    def save_waypoints(self):
        """Salvează waypoint-uri în YAML"""
        try:
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f, default_flow_style=False, sort_keys=False)
            return True
        except Exception as e:
            self.get_logger().error(f'Eroare salvare: {e}')
            return False
    
    # ==================== RECORDER MODE ====================
    
    def setup_recorder(self):
        """Configurare mod înregistrare waypoint-uri"""
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw_deg = 0.0
        self.pose_received = False
        
        # Subscriber pentru poziția de la AMCL
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # Timer pentru afișare poziție
        self.display_timer = self.create_timer(0.5, self.display_position)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('🎯 WAYPOINT RECORDER - Mod înregistrare pornit')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Fișier: {self.waypoints_file}')
        self.get_logger().info(f'Waypoint-uri încărcate: {len(self.waypoints)}')
        self.get_logger().info('')
        self.get_logger().info('COMENZI:')
        self.get_logger().info('  [Enter] - Salvează poziția curentă ca waypoint')
        self.get_logger().info('  [Ctrl+C] - Ieșire')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
    
    def pose_callback(self, msg):
        """Primește poziția curentă de la AMCL"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convertește quaternion în yaw
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw_deg = math.degrees(yaw)
        
        self.pose_received = True
    
    def display_position(self):
        """Afișează poziția curentă periodic"""
        if self.pose_received:
            print(f'\r📍 Poziție: X={self.current_x:.3f}m  Y={self.current_y:.3f}m  Yaw={self.current_yaw_deg:.1f}°  ', end='', flush=True)
    
    def record_waypoint(self):
        """Salvează waypoint-ul curent"""
        if not self.pose_received:
            print('\n❌ Nu am primit încă date de la AMCL! Verifică că navxplorer.launch.py rulează.')
            return
        
        print('\n\n' + '=' * 70)
        waypoint_name = input('📝 Nume waypoint (ex: hol, bucatarie, birou): ').strip()
        
        if not waypoint_name:
            print('❌ Nume invalid. Anulat.')
            return
        
        if waypoint_name in self.waypoints:
            confirm = input(f'⚠️  Waypoint "{waypoint_name}" există deja. Suprascriu? (da/nu): ').strip().lower()
            if confirm not in ['da', 'd', 'yes', 'y']:
                print('❌ Anulat.')
                return
        
        # Salvează waypoint-ul
        self.waypoints[waypoint_name] = {
            'x': round(self.current_x, 3),
            'y': round(self.current_y, 3),
            'yaw': round(self.current_yaw_deg, 1)
        }
        
        if self.save_waypoints():
            print(f'✅ Waypoint "{waypoint_name}" salvat: ({self.current_x:.3f}, {self.current_y:.3f}, {self.current_yaw_deg:.1f}°)')
            print(f'📊 Total waypoint-uri: {len(self.waypoints)}')
        else:
            print('❌ Eroare la salvare!')
        
        print('=' * 70)
        print()
    
    # ==================== NAVIGATOR MODE ====================
    
    def setup_navigator(self):
        """Configurare mod navigație"""
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None
        self.is_navigating = False
        
        self.get_logger().info('🚀 Navigator pornit!')
        self.get_logger().info(f'Waypoint-uri disponibile: {list(self.waypoints.keys())}')
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convertește Euler în quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]
    
    def cancel_current_navigation(self):
        """Anulează navigația curentă"""
        if self.current_goal_handle is not None and self.is_navigating:
            self.get_logger().info('🛑 Anulez navigația curentă...')
            self.current_goal_handle.cancel_goal_async()
            self.is_navigating = False
    
    def navigate_to(self, waypoint_name):
        """Navigare către waypoint"""
        if waypoint_name not in self.waypoints:
            self.get_logger().error(f'❌ Waypoint "{waypoint_name}" nu există!')
            self.get_logger().info(f'Disponibile: {list(self.waypoints.keys())}')
            return False
        
        # Anulează navigația anterioară dacă există
        if self.is_navigating:
            self.cancel_current_navigation()
        
        wp = self.waypoints[waypoint_name]
        x, y, yaw_deg = wp['x'], wp['y'], wp['yaw']
        yaw_rad = math.radians(yaw_deg)
        
        self.get_logger().info(f'🎯 Navighez la "{waypoint_name}" → ({x:.2f}, {y:.2f}, {yaw_deg}°)')
        
        # Așteaptă action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 nu răspunde! Pornește navxplorer.launch.py')
            return False
        
        self.is_navigating = True
        
        # Creează goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        quat = self.quaternion_from_euler(0, 0, yaw_rad)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        # Trimite goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal respins!')
            self.is_navigating = False
            return
        
        self.current_goal_handle = goal_handle
        self.get_logger().info('✅ Goal acceptat! Robotul navighează...')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        # Poți decomenta pentru distanță rămasă:
        # distance = feedback_msg.feedback.distance_remaining
        # self.get_logger().info(f'📍 {distance:.2f}m')
        pass
    
    def result_callback(self, future):
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Ajuns la destinație!')
        elif status == 5:  # ABORTED
            self.get_logger().warn('⚠️  Navigație abandonată (obstacol/eroare)')
        elif status == 6:  # CANCELED
            self.get_logger().warn('🛑 Navigație anulată')
        else:
            self.get_logger().error(f'❌ Eșec (status: {status})')
        
        self.current_goal_handle = None
        self.is_navigating = False

# ==================== MAIN ====================

def main():
    rclpy.init()
    
    # Dacă nu are argumente, intră în mod interactiv
    if len(sys.argv) < 2:
        print('\n' + '=' * 70)
        print('🎯 WAYPOINT NAVIGATOR - Sistem de navigație')
        print('=' * 70)
        
        # Încarcă waypoint-uri pentru listare
        waypoints_file = os.path.expanduser(
            '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml'
        )
        
        waypoints = {}
        if os.path.exists(waypoints_file):
            try:
                with open(waypoints_file, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data:
                        waypoints = data['waypoints']
            except:
                pass
        
        print('\nComenzi speciale:')
        print('  record  - Înregistrează waypoint-uri noi')
        print('  list    - Listează waypoint-urile salvate')
        print('  stop    - Oprește navigația curentă')
        
        if waypoints:
            print('\n📍 Waypoint-uri disponibile:')
            for name in waypoints.keys():
                print(f'  • {name}')
        else:
            print('\n⚠️  Niciun waypoint salvat încă. Folosește "record" pentru a crea.')
        
        print('=' * 70)
        
        command = input('\n🤖 Unde să merg? (sau "record"/"list"): ').strip()
        
        if not command:
            print('❌ Nicio comandă introdusă.')
            sys.exit(1)
    else:
        command = sys.argv[1]
    
    # Mod listare
    if command == 'list':
        waypoints_file = os.path.expanduser(
            '~/dummybot-ros2-encoder/ros2_ws/src/amr2ax_nav2/config/named_waypoints.yaml'
        )
        if os.path.exists(waypoints_file):
            with open(waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                if data and 'waypoints' in data:
                    print('\n📍 Waypoint-uri salvate:')
                    print('=' * 60)
                    for name, wp in data['waypoints'].items():
                        print(f"  {name:15} → ({wp['x']:6.2f}, {wp['y']:6.2f}, {wp['yaw']:6.1f}°)")
                    print('=' * 60)
                else:
                    print('❌ Fișierul e gol.')
        else:
            print('❌ Niciun waypoint salvat încă.')
        sys.exit(0)
    
    # Mod înregistrare
    if command == 'record':
        node = WaypointNavigator(mode='record')
        
        # Așteaptă primul mesaj (AMCL publică la ~1.5 Hz, deci așteptăm mai mult)
        import time
        start = time.time()
        timeout = 15.0
        
        node.get_logger().info('⏳ Aștept date de la AMCL (poate dura până la 15s)...')
        
        while not node.pose_received and (time.time() - start) < timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
        
        if not node.pose_received:
            print('\n❌ Nu am primit date de la AMCL după 15 secunde!')
            print('Verifică că navxplorer.launch.py rulează.')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        
        print('✅ Conectat! Vezi poziția LIVE mai jos.\n')
        
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                # Așteaptă Enter pentru salvare
                import select
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    node.record_waypoint()
        except KeyboardInterrupt:
            print('\n\n🛑 Închidere recorder...')
        finally:
            node.destroy_node()
            rclpy.shutdown()
    
    # Mod navigație interactiv
    else:
        node = WaypointNavigator(mode='navigate')
        
        # Thread pentru ROS spin
        def ros_spin_thread():
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
        
        spin_thread = threading.Thread(target=ros_spin_thread, daemon=True)
        spin_thread.start()
        
        # Procesează prima comandă
        interactive_mode = len(sys.argv) < 2
        waypoint_name = command
        
        try:
            while True:
                if waypoint_name == 'stop':
                    node.cancel_current_navigation()
                elif waypoint_name:
                    node.navigate_to(waypoint_name)
                
                # Dacă nu e mod interactiv, ieși
                if not interactive_mode:
                    # Așteaptă să termine navigația
                    while node.is_navigating and rclpy.ok():
                        import time
                        time.sleep(0.1)
                    break
                
                # Reîncarcă waypoint-urile
                node.waypoints = node.load_waypoints()
                
                # Cere următoarea comandă
                print('\n' + '=' * 70)
                print('📍 Waypoint-uri:', list(node.waypoints.keys()))
                waypoint_name = input('🤖 Comandă (waypoint/stop/Enter=ieșire): ').strip()
                
                if not waypoint_name:
                    print('👋 La revedere!')
                    node.cancel_current_navigation()
                    break
        
        except KeyboardInterrupt:
            print('\n🛑 Oprire')
            node.cancel_current_navigation()
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()