#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleOdometry, VehicleCommandAck
from geometry_msgs.msg import PoseStamped
import numpy as np


class OffboardControl(Node):

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_profile)
        
        # Dynamic waypoint subscriber (after takeoff)
        self.create_subscription(
            PoseStamped, '/offboard_setpoint_pose', self.dynamic_waypoint_callback, 10)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_command_ack = VehicleCommandAck()

        self.takeoff_height = -30.0
        self.offboard_setpoint_counter = 0
        self.taken_off = None

        self.current_dynamic_pose = None
        self.tracking_dynamic_waypoint = False

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def vehicle_command_ack_callback(self, msg):
        self.vehicle_command_ack = msg

    def dynamic_waypoint_callback(self, msg: PoseStamped):
        if self.taken_off:  # Only accept waypoints after takeoff
            self.current_dynamic_pose = msg
            self.tracking_dynamic_waypoint = True
            self.get_logger().info(f"Received dynamic waypoint: ({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 1.57079):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]} with yaw {yaw:.2f}")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def compute_yaw_towards_target(self, x_target, y_target):
        dx = x_target - self.vehicle_local_position.x
        dy = y_target - self.vehicle_local_position.y
        return np.arctan2(dy, dx)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if (self.vehicle_local_position.z > self.takeoff_height and
            self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and
            self.taken_off is None):
            print("publishing takeoff")
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            if self.vehicle_local_position.z < self.takeoff_height + 0.5:
                self.taken_off = True
                print("taken off")

        # After takeoff: follow dynamic waypoints with yaw aligned to direction
        if self.taken_off and self.tracking_dynamic_waypoint and self.current_dynamic_pose:
            x = self.current_dynamic_pose.pose.position.x
            y = self.current_dynamic_pose.pose.position.y
            z = self.current_dynamic_pose.pose.position.z

            yaw = self.compute_yaw_towards_target(x, y)
            self.publish_position_setpoint(x, y, z, yaw)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None):
    print('Starting offboard control node...')
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
