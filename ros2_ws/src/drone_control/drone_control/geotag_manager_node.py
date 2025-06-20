import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition
from geographiclib.geodesic import Geodesic
import numpy as np


class GPSToLocalPose(Node):
    def __init__(self):
        super().__init__('gps_to_local_pose_node')

        # Reference origin
        self.origin_set = False
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        # Target local position (converted from GPS)
        self.target_position = None
        self.reached_threshold = 1.0  # meters

        # Subscriptions
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, 10)
        self.create_subscription(Float64MultiArray, '/target_gps', self.gps_target_callback, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/offboard_setpoint_pose', 10)
        self.status_pub = self.create_publisher(String, '/drone_status', 10)

        # Timer to check if drone has reached the target
        self.timer = self.create_timer(0.5, self.check_if_reached)

        self.current_position = None

    def local_position_callback(self, msg):
        if not self.origin_set and msg.xy_valid:
            self.origin_lat = msg.ref_lat * 1e-7
            self.origin_lon = msg.ref_lon * 1e-7
            self.origin_alt = msg.ref_alt * 1e-3
            self.origin_set = True
            self.get_logger().info(f"[Origin Set] lat={self.origin_lat}, lon={self.origin_lon}, alt={self.origin_alt}")

        # Update current position
        if msg.xy_valid:
            self.current_position = np.array([msg.x, msg.y, msg.z])

    def gps_target_callback(self, msg):
        if not self.origin_set:
            self.get_logger().warn("Origin not yet set. Can't convert GPS to local.")
            return

        target_lat, target_lon, target_alt = msg.data

        # Convert GPS to local
        geo = Geodesic.WGS84.Inverse(self.origin_lat, self.origin_lon, target_lat, target_lon)
        x = geo['s12'] * np.sin(np.radians(geo['azi1']))  # East
        y = geo['s12'] * np.cos(np.radians(geo['azi1']))  # North
        z = self.origin_alt - target_alt  # NED Down (z increases downward)

        self.target_position = np.array([x, y, -z])  # ENU: Z positive up

        self.get_logger().info(f"Target local position: x={x:.2f}, y={y:.2f}, z={-z:.2f}")

        # Publish PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = -z  # Convert NED -> ENU (Z up)

        pose.pose.orientation.w = 1.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0

        self.pose_pub.publish(pose)

    def check_if_reached(self):
        if self.target_position is None or self.current_position is None:
            return

        distance = np.linalg.norm(self.current_position[:2] - self.target_position[:2])

        if distance < self.reached_threshold:
            msg = String()
            msg.data = "reached"
            self.status_pub.publish(msg)
            self.get_logger().info(f"[STATUS] Reached target (distance={distance:.2f}m).")
            self.target_position = None  # Prevent repeated publishing


def main(args=None):
    rclpy.init(args=args)
    node = GPSToLocalPose()
    rclpy.spin(node)
    rclpy.shutdown()
