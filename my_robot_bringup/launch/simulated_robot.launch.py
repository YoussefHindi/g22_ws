import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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
    
    teleop = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            'export TURTLEBOT3_MODEL=burger; ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped'
        ],
        name='teleop_keyboard'
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("my_robot_mapping"),
            "launch",
            "slam.launch.py"
        ),
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
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        slam,
        rviz_slam,
        teleop,

    ])