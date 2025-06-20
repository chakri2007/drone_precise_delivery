from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Aruco tracker node
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('aruco_tracker'), 'cfg', 'params.yaml'])
            ]
        ),
        Node(
            package='drone_control',
            executable='drone_controller_node',
            name='drone_controller_node',
            output='screen',
        ),
        Node(
            package='drone_control',
            executable='offboard_controller_node',
            name='offboard_controller_node',
            output='screen',
        ),
        Node(
            package='drone_control',
            executable='geotag_manager_node',
            name='geotag_manager_node',
            output='screen',
        ),
    ])