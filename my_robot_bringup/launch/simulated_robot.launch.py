import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    my_robot_controller_pkg = get_package_share_directory('my_robot_controller')

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    
    teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            'export TURTLEBOT3_MODEL=burger; ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped'
        ],
        name='teleop_keyboard'
    )

    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "diff_drive_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(my_robot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(my_robot_controller_pkg, "config", "twist_mux_topics.yaml"),
            # "config_joy": os.path.join(my_robot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("my_robot_localization"),
                "rviz",
                "global_localization.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam)
    )


    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("my_robot_mapping"),
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        twist_mux_launch,
        localization,
        slam,
        rviz_slam,
        rviz_localization,
        teleop,

    ])