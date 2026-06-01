from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_hat_driver",
            executable="robot_hat_node",
            name="robot_hat_node",
            parameters=[
                "config/robot_hat.yaml"
            ],
        )
    ])