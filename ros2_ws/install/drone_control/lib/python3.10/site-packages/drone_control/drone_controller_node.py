import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLandDetected
from std_msgs.msg import String

import numpy as np
from dataclasses import dataclass


@dataclass
class ArucoTag:
    position: np.ndarray
    orientation: np.ndarray  # quaternion [w, x, y, z]
    timestamp: float


class PrecisionLand(Node):
    def __init__(self):
        super().__init__('precision_land_node')

        self.state = "Idle"
        self._tag = None
        self._search_started = False
        self._land_detected = False
        self._vel_x_integral = 0.0
        self._vel_y_integral = 0.0
        self._search_waypoints = []
        self._search_waypoint_index = 0
        self._package_dropped = False

        self.declare_parameters('', [
           ('descent_vel', 0.6),
           ('vel_p_gain', 1.7),
           ('vel_i_gain', 0.0),
           ('max_velocity', 3.0),
           ('target_timeout', 5.0),
           ('delta_position', 0.2),
           ('delta_velocity', 0.2),
        ])

        self.descent_vel = self.get_parameter('descent_vel').get_parameter_value().double_value
        self.vel_p_gain = self.get_parameter('vel_p_gain').get_parameter_value().double_value
        self.vel_i_gain = self.get_parameter('vel_i_gain').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.target_timeout = self.get_parameter('target_timeout').get_parameter_value().double_value
        self.delta_position = self.get_parameter('delta_position').get_parameter_value().double_value
        self.delta_velocity = self.get_parameter('delta_velocity').get_parameter_value().double_value

        self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)
        self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.land_detected_callback, 10)
        self.create_subscription(String, '/drone_status', self.status_callback, 10)

        self.timer = self.create_timer(0.1, self.update)

    def status_callback(self, msg):
        if msg.data.lower() == "reached" and self.state == "Idle":
            self.get_logger().info("Received status 'reached'. Switching to Search state.")
            self.state = "Search"
            self._search_started = True

    def target_pose_callback(self, msg):
        if not self._search_started:
            return

        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
        self._tag = ArucoTag(position, orientation, self.get_clock().now().nanoseconds / 1e9)

    def land_detected_callback(self, msg):
        self._land_detected = msg.landed

    def update(self):
        if self.state == "Idle":
            return

        if self.state == "Search":
            self.search_behavior()

        elif self.state == "Approach":
            self.approach_behavior()

        elif self.state == "Descend":
            self.descend_behavior()

    def search_behavior(self):
        if self._tag and not self.check_target_timeout():
            self.state = "Approach"
            self.get_logger().info("Target found. Switching to Approach.")
            return

        if not self._search_waypoints:
            self.generate_search_waypoints()

        current_wp = self._search_waypoints[self._search_waypoint_index]
        self.get_logger().info(f"Navigating to search waypoint {self._search_waypoint_index}: {current_wp}")

        if np.linalg.norm(current_wp[:2]) < self.delta_position:
            self._search_waypoint_index = (self._search_waypoint_index + 1) % len(self._search_waypoints)

    def approach_behavior(self):
        if self.check_target_timeout():
            self.state = "Idle"
            self.get_logger().info("Target lost during approach.")
            return

        target_xy = self._tag.position[:2]
        self.get_logger().info(f"Approaching target at XY: {target_xy}")
        if np.linalg.norm(target_xy) < self.delta_position:
            self.state = "Descend"

    def descend_behavior(self):
        if self.check_target_timeout():
            self.state = "Idle"
            self.get_logger().info("Target lost during descend.")
            return

        target_altitude_above_tag = 6.1  # 20 feet

        current_z = 0.0  # Replace with actual altitude if available
        tag_z = self._tag.position[2]
        relative_altitude = current_z - tag_z

        if not self._package_dropped and relative_altitude <= target_altitude_above_tag:
            self.drop_package()
            self._package_dropped = True
            self.state = "Idle"
            return

        vel = self.calculate_velocity_setpoint_xy()
        self.get_logger().info(f"Descending to drop altitude. Velocity: {vel}")

    def drop_package(self):
        self.get_logger().info("Dropping package engaged.")

    def calculate_velocity_setpoint_xy(self):
        delta_x = -self._tag.position[0]
        delta_y = -self._tag.position[1]

        self._vel_x_integral += delta_x
        self._vel_y_integral += delta_y

        self._vel_x_integral = np.clip(self._vel_x_integral, -self.max_velocity, self.max_velocity)
        self._vel_y_integral = np.clip(self._vel_y_integral, -self.max_velocity, self.max_velocity)

        vx = self.vel_p_gain * delta_x + self.vel_i_gain * self._vel_x_integral
        vy = self.vel_p_gain * delta_y + self.vel_i_gain * self._vel_y_integral

        vx = np.clip(vx, -self.max_velocity, self.max_velocity)
        vy = np.clip(vy, -self.max_velocity, self.max_velocity)

        return np.array([vx, vy, self.descent_vel])

    def check_target_timeout(self):
        if self._tag is None:
            return True
        return (self.get_clock().now().nanoseconds / 1e9 - self._tag.timestamp) > self.target_timeout

    def generate_search_waypoints(self):
        waypoints = []
        radius = 2.0
        points_per_layer = 16
        layer_spacing = 0.5
        current_z = -1.0

        for layer in range(2):
            layer_wps = []
            for i in range(points_per_layer):
                angle = 2 * np.pi * i / points_per_layer
                x = radius * np.cos(angle)
                y = radius * np.sin(angle)
                layer_wps.append(np.array([x, y, current_z]))
                radius -= radius / points_per_layer
            waypoints.extend(layer_wps)
            current_z += layer_spacing

        self._search_waypoints = waypoints


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLand()
    rclpy.spin(node)
    rclpy.shutdown()
