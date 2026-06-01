from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="sunfounder_robot_hat",
            executable="robot_hat_node",
            name="robot_hat",
            parameters=[
                "config/robot_hat.yaml"
            ],
        )
    ])