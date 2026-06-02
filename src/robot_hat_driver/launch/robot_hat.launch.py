from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_hat_driver",
            executable="rhdriver",
            name="robot_hat_node",
            parameters=[
                PathJoinSubstitution([FindPackageShare('robot_hat_driver'), 'config', 'robot_hat.yaml'])
            ],
        )
    ])