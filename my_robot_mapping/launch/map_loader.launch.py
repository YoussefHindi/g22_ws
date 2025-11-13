import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")


    lifecycle_nodes = ["map_server"]

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory("my_robot_mapping"),
        "maps",
        "small_house",
        "map.yaml"
    ])

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_map_server,
        nav2_lifecycle_manager,
    ])
