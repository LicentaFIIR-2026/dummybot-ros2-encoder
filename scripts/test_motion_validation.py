#!/usr/bin/env python3
"""
Test complet de validare a subsistemului de propulsie.

Ruleaza secvential:
  1. Translatie 1m inainte
  2. Pauza pentru repozitionare (optional)
  3. Rotatie 360deg (2*pi rad)

Pentru fiecare test, capteaza:
  - Setpoint comandat
  - Odometrie (/odom): pozitie si orientare
  - Joint states (/joint_states): pozitia radiana acumulata per roata

La final afiseaza raport consolidat cu cele trei surse de adevar:
  setpoint  vs  odometrie  vs  integrare encodere
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import time

# Parametri fizici ai robotului
WHEEL_RADIUS = 0.056       # 56 mm = raza rotii (diametru 112 mm)
WHEEL_SEPARATION = 0.30    # distanta intre rotile stanga si dreapta (ajusteaza daca e altfel)

# Parametri test translatie
LINEAR_TARGET = 1.0        # m
LINEAR_VELOCITY = 0.2      # m/s

# Parametri test rotatie
ANGULAR_TARGET = 2 * math.pi   # rad (360 deg)
ANGULAR_VELOCITY = 0.5         # rad/s


class MotionValidationTest(Node):
    def __init__(self):
        super().__init__('motion_validation_test')

        # Publisher pe topic-ul real al controller-ului
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscribere
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # State odometrie
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False

        # State joint states
        self.joint_names = []
        self.joint_positions = []
        self.joints_ready = False

        self.get_logger().info('Node initializat. Astept odometrie si joint_states...')

    # ---------- Callbacks ----------
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion -> yaw (rotatie in plan XY)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        if not self.odom_ready:
            self.odom_ready = True

    def joint_callback(self, msg):
        self.joint_names = list(msg.name)
        self.joint_positions = list(msg.position)
        if not self.joints_ready:
            self.joints_ready = True

    # ---------- Helpers ----------
    def wait_for_data(self):
        while not (self.odom_ready and self.joints_ready):
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.3)  # stabilizare

    def send_velocity(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        for _ in range(10):
            self.send_velocity(0.0, 0.0)
            time.sleep(0.05)

    def snapshot(self):
        """Captura stare curenta (pentru comparatie start vs final)."""
        return {
            'x': self.current_x,
            'y': self.current_y,
            'yaw': self.current_yaw,
            'joints': dict(zip(self.joint_names, self.joint_positions)),
        }

    def distance_2d(self, a, b):
        return math.sqrt((b['x'] - a['x'])**2 + (b['y'] - a['y'])**2)

    def yaw_diff(self, a, b):
        """Diferenta de yaw, gestionand wrap-around."""
        diff = b['yaw'] - a['yaw']
        # daca diff e mic dar real a fost ~2pi, e nevoie sa "unwrap"
        # facem unwrap manual: daca |diff| < pi/2 dar test a fost ~2pi,
        # asumam ca s-a facut o rotatie completa
        return diff

    def joints_delta(self, start, end):
        """Diferenta radiani per joint intre start si end."""
        deltas = {}
        for name in start['joints']:
            if name in end['joints']:
                deltas[name] = end['joints'][name] - start['joints'][name]
        return deltas

    # ---------- Test 1: Translatie ----------
    def run_translation_test(self):
        print('\n' + '=' * 70)
        print('TEST 1: TRANSLATIE')
        print('=' * 70)
        print(f'  Setpoint:  {LINEAR_TARGET:.3f} m')
        print(f'  Viteza:    {LINEAR_VELOCITY:.2f} m/s')
        print(f'  Durata teoretica: {LINEAR_TARGET / LINEAR_VELOCITY:.2f} s')
        print('=' * 70)
        input('Pune robotul intr-un spatiu liber (>1.5m in fata) si apasa ENTER...')

        start = self.snapshot()
        print(f'\n[START] pos=({start["x"]:.3f}, {start["y"]:.3f}) yaw={math.degrees(start["yaw"]):.2f}deg')
        for name in start['joints']:
            print(f'        {name}: {start["joints"][name]:.4f} rad')

        print('\nPornesc miscarea...')
        last_print = 0.0
        while rclpy.ok():
            self.send_velocity(LINEAR_VELOCITY, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            now = self.snapshot()
            dist = self.distance_2d(start, now)
            if dist - last_print >= 0.1:
                print(f'  ... {dist:.3f} m')
                last_print = dist
            if dist >= LINEAR_TARGET:
                break

        self.stop_robot()
        time.sleep(0.5)
        end = self.snapshot()

        # Analiza
        dist_odom = self.distance_2d(start, end)
        joint_deltas = self.joints_delta(start, end)
        # distanta din fiecare roata (rad * raza)
        per_wheel_distances = {n: d * WHEEL_RADIUS for n, d in joint_deltas.items()}
        avg_wheel_distance = sum(per_wheel_distances.values()) / len(per_wheel_distances)

        print('\n' + '-' * 70)
        print('REZULTAT TRANSLATIE')
        print('-' * 70)
        print(f'  Setpoint comandat:             {LINEAR_TARGET:.4f} m')
        print(f'  Distanta din /odom:            {dist_odom:.4f} m')
        print(f'  Distanta medie din encodere:   {avg_wheel_distance:.4f} m')
        print()
        print('  Detaliu per roata:')
        for name, delta_rad in joint_deltas.items():
            delta_m = delta_rad * WHEEL_RADIUS
            print(f'    {name}: {delta_rad:+.4f} rad ({delta_m:+.4f} m)')
        print()
        err_odom = (dist_odom - LINEAR_TARGET) / LINEAR_TARGET * 100
        err_enc = (avg_wheel_distance - LINEAR_TARGET) / LINEAR_TARGET * 100
        err_odom_vs_enc = (dist_odom - avg_wheel_distance) / avg_wheel_distance * 100 if avg_wheel_distance != 0 else 0
        print(f'  Eroare odom vs setpoint:       {err_odom:+.2f} %')
        print(f'  Eroare encodere vs setpoint:   {err_enc:+.2f} %')
        print(f'  Diferenta odom vs encodere:    {err_odom_vs_enc:+.2f} %')
        print('-' * 70)

        return {
            'setpoint': LINEAR_TARGET,
            'odom': dist_odom,
            'encoders_avg': avg_wheel_distance,
            'joint_deltas': joint_deltas,
            'start': start,
            'end': end,
        }

    # ---------- Test 2: Rotatie ----------
    def run_rotation_test(self):
        print('\n' + '=' * 70)
        print('TEST 2: ROTATIE')
        print('=' * 70)
        print(f'  Setpoint:  {ANGULAR_TARGET:.3f} rad ({math.degrees(ANGULAR_TARGET):.1f} deg)')
        print(f'  Viteza:    {ANGULAR_VELOCITY:.2f} rad/s')
        print(f'  Durata teoretica: {ANGULAR_TARGET / ANGULAR_VELOCITY:.2f} s')
        print('=' * 70)
        input('Pune robotul intr-un spatiu liber (>0.5m raza) si apasa ENTER...')

        start = self.snapshot()
        print(f'\n[START] pos=({start["x"]:.3f}, {start["y"]:.3f}) yaw={math.degrees(start["yaw"]):.2f}deg')
        for name in start['joints']:
            print(f'        {name}: {start["joints"][name]:.4f} rad')

        print('\nPornesc rotatia...')
        # pentru rotatie urmarim yaw cumulativ (cu unwrap)
        last_yaw = start['yaw']
        cumulative_yaw = 0.0
        last_print = 0.0
        while rclpy.ok():
            self.send_velocity(0.0, ANGULAR_VELOCITY)
            rclpy.spin_once(self, timeout_sec=0.01)

            # unwrap manual pentru yaw
            d_yaw = self.current_yaw - last_yaw
            if d_yaw > math.pi:
                d_yaw -= 2 * math.pi
            elif d_yaw < -math.pi:
                d_yaw += 2 * math.pi
            cumulative_yaw += d_yaw
            last_yaw = self.current_yaw

            if abs(cumulative_yaw) - last_print >= math.radians(30):
                print(f'  ... {math.degrees(abs(cumulative_yaw)):.1f} deg')
                last_print = abs(cumulative_yaw)

            if abs(cumulative_yaw) >= ANGULAR_TARGET:
                break

        self.stop_robot()
        time.sleep(0.5)
        end = self.snapshot()

        # Analiza
        joint_deltas = self.joints_delta(start, end)
        # distanta de arc per roata (rad * raza)
        per_wheel_arcs = {n: d * WHEEL_RADIUS for n, d in joint_deltas.items()}
        # rotatie estimata din diff drive: (right - left) / wheel_separation
        # pentru 4 roti: media stanga si media dreapta
        left_joints = [n for n in joint_deltas if 'left' in n.lower()]
        right_joints = [n for n in joint_deltas if 'right' in n.lower()]
        avg_left_arc = sum(per_wheel_arcs[n] for n in left_joints) / max(len(left_joints), 1)
        avg_right_arc = sum(per_wheel_arcs[n] for n in right_joints) / max(len(right_joints), 1)
        rotation_from_encoders = (avg_right_arc - avg_left_arc) / WHEEL_SEPARATION

        print('\n' + '-' * 70)
        print('REZULTAT ROTATIE')
        print('-' * 70)
        print(f'  Setpoint comandat:                  {ANGULAR_TARGET:.4f} rad ({math.degrees(ANGULAR_TARGET):.2f} deg)')
        print(f'  Rotatie cumulativa din /odom:       {cumulative_yaw:.4f} rad ({math.degrees(cumulative_yaw):.2f} deg)')
        print(f'  Rotatie estimata din encodere:      {rotation_from_encoders:.4f} rad ({math.degrees(rotation_from_encoders):.2f} deg)')
        print()
        print('  Detaliu per roata:')
        for name, delta_rad in joint_deltas.items():
            arc_m = delta_rad * WHEEL_RADIUS
            print(f'    {name}: {delta_rad:+.4f} rad (arc {arc_m:+.4f} m)')
        print()
        err_odom = (cumulative_yaw - ANGULAR_TARGET) / ANGULAR_TARGET * 100
        err_enc = (rotation_from_encoders - ANGULAR_TARGET) / ANGULAR_TARGET * 100
        print(f'  Eroare odom vs setpoint:            {err_odom:+.2f} %')
        print(f'  Eroare encodere vs setpoint:        {err_enc:+.2f} %')
        print('-' * 70)

        return {
            'setpoint': ANGULAR_TARGET,
            'odom': cumulative_yaw,
            'encoders': rotation_from_encoders,
            'joint_deltas': joint_deltas,
            'start': start,
            'end': end,
        }

    # ---------- Raport final ----------
    def print_final_report(self, trans_result, rot_result):
        print('\n\n' + '=' * 70)
        print('RAPORT CONSOLIDAT')
        print('=' * 70)
        print()
        print('TRANSLATIE 1 m')
        print(f'  Setpoint:     {trans_result["setpoint"]:.4f} m')
        print(f'  Odom:         {trans_result["odom"]:.4f} m  ({(trans_result["odom"] - trans_result["setpoint"]) / trans_result["setpoint"] * 100:+.2f} %)')
        print(f'  Encodere:     {trans_result["encoders_avg"]:.4f} m  ({(trans_result["encoders_avg"] - trans_result["setpoint"]) / trans_result["setpoint"] * 100:+.2f} %)')
        print()
        print(f'ROTATIE 360 deg ({ANGULAR_TARGET:.3f} rad)')
        print(f'  Setpoint:     {math.degrees(rot_result["setpoint"]):.2f} deg')
        print(f'  Odom:         {math.degrees(rot_result["odom"]):.2f} deg  ({(rot_result["odom"] - rot_result["setpoint"]) / rot_result["setpoint"] * 100:+.2f} %)')
        print(f'  Encodere:     {math.degrees(rot_result["encoders"]):.2f} deg  ({(rot_result["encoders"] - rot_result["setpoint"]) / rot_result["setpoint"] * 100:+.2f} %)')
        print()
        print('=' * 70)


def main():
    rclpy.init()
    node = MotionValidationTest()

    try:
        print('Astept date pe /odom si /joint_states...')
        node.wait_for_data()
        print('Date primite.')

        trans = node.run_translation_test()

        print('\n[PAUZA] Repozitioneaza robotul daca vrei.')
        input('Apasa ENTER pentru a porni testul de rotatie...')

        rot = node.run_rotation_test()

        node.print_final_report(trans, rot)

    except KeyboardInterrupt:
        print('\nTest intrerupt!')
        node.stop_robot()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
