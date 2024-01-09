#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # Subscriber do skanera laserowego
        self.laser_scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )

        # Subscriber odległości do celu
        self.distance_subscriber = self.create_subscription(
            Float32,
            '/referee/distance_left',
            self.distance_callback,
            10
        )

        # Publisher dla komend ruchu
        self.movement_command_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Cel robotów
        self.goal_reached = False
        self.state = "straight"  # Stan początkowy - robot jedzie prosto

    def laser_scan_callback(self, msg):
        if not self.goal_reached:
            # Algorytm DWA
            desired_velocity = self.calculate_desired_velocity()
            allowable_v, allowable_w = self.generate_window()

            best_v, best_w = self.find_optimal_velocity(desired_velocity, allowable_v, allowable_w, msg)

            # Wyślij komendę ruchu do robota
            self.publish_movement_command(best_v, best_w)


    def distance_callback(self, msg):
        try:
            # Sprawdź czy osiągnięto cel
            distance_left = msg.data
            if distance_left <= 0.3:
                self.goal_reached = True
                self.get_logger().info('Cel osiągnięty!')
        # Próba "osiągnięcia celu" w momencie gdy topic znika
        except Exception as e:
            self.goal_reached = True
            self.get_logger().info('Cel osiągnięty!')

    def calculate_desired_velocity(self):
        return 0.1  # Dla uproszczenia - stała prędkość

    def generate_window(self):
        allowable_v = [0.1, 0.3, 0.5]  # Okno - prędkości liniowe
        allowable_w = [-0.3, 0.0, 0.3]  # Okno - prędkości kątowe
        return allowable_v, allowable_w

    def find_optimal_velocity(self, desired_velocity, allowable_v, allowable_w, laser_scan):
        optimal_cost = float('-inf')
        best_v, best_w = 0.0, 0.0

        min_distance_to_obstacle = 0.15
        turn_distance_threshold = 0.15 
        min_turning_distance = 0.25 
        safety_margin = 0.05 
        turning_duration = 0
        max_turning_duration = 5 

        for v in allowable_v:
            for w in allowable_w:
                dist = self.find_distance(v, w, laser_scan)
                print(f"dist: {dist}")
                break_dist = self.calculate_breaking_distance(v)
                print(f"break_dist: {break_dist}")
                print(f"state: {self.state}")

                # Zacznij obrót robotem kiedy ściana jest za blisko a robot nie jest w stanie obrotu
                if 0 < dist < (turn_distance_threshold - safety_margin) and self.state != "turning":
                    print(f"Start turning when the obstacle is too close")
                    self.state = "turning"  # Stan - turning
                    print(f"state: {self.state}")
                    best_v, best_w = 0.0, 0.5  # Prędkość obrotu robota w miejscu bez prędkości liniowej
                    optimal_cost = float('inf') 
                    print(f"best_v: {best_v}, best_w: {best_w}")

                elif self.state == "turning":
                    print(f"state: {self.state}")

                    if dist > min_turning_distance:
                        best_v, best_w = 0.0, 0.1 
                        optimal_cost = float('inf')  
                    else:
                        best_v, best_w = 0.0, 0.3 
                    print(f"best_v: {best_v}, best_w: {best_w}")

                    # Powrót do ruchu po prostej, gdy przeszkoda jest odpowiednio daleko 
                    if dist >= min_distance_to_obstacle + 0.05:
                        print(f"Switching back to straight after turning, dist: {dist}")
                        self.state = "straight"
                        print(f"state: {self.state}")

                elif self.state == "straight" and dist > min_distance_to_obstacle and dist > turn_distance_threshold:
                    heading = self.calculate_heading(v, w)
                    print(f"heading: {heading}")
                    clearance = (dist - break_dist) / (max(laser_scan.ranges) - break_dist)
                    cost = self.calculate_cost(heading, clearance, abs(desired_velocity - v))
                    if cost > optimal_cost:
                        self.state = "straight"
                        best_v, best_w = v, w
                        optimal_cost = cost
                        print(f"best_v: {best_v}, best_w: {best_w}")

                elif self.state == "straight" and dist < min_distance_to_obstacle and dist < turn_distance_threshold and dist < min_turning_distance:
                    print(f"Switching to turning from straight, dist: {dist}")
                    self.state = "turning"
                    print(f"state: {self.state}")



        return best_v, best_w


    def find_distance(self, v, w, laser_scan):
        # Minimalna odległość z odczytu skanu laserowego
        return min(laser_scan.ranges)

    def calculate_breaking_distance(self, v):
        # Odległość hamowania - 10% prędkości liniowej
        return 0.1*v 

    def calculate_heading(self, v, w):
        # Kierunek
        return math.atan2(w, v)

    def calculate_cost(self, heading, clearance, velocity_difference):
        # Funkcja kosztu
        return math.cos(heading) + 0.5 * clearance - 0.2 * velocity_difference

    # Ruch
    def publish_movement_command(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.movement_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    navigator_node = NavigatorNode()
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

