from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            executable="talker",
            package="publisher_subscriber_pkg",
            name="hello_node",
        ),
        Node(
            executable="listener",
            package="publisher_subscriber_pkg",
            name="sub_node",
        )
    ])