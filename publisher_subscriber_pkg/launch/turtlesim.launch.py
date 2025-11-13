from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            executable="turtlesim_node",
            package="turtlesim",
            name="turtlsim_node",
        ),
        Node(
            executable="turtle_mover",
            package="publisher_subscriber_pkg",
            name="moving_in_circle",
        )
    ])