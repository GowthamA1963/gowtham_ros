import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Allow /use_sim_time to be set at launch time
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    pkg = get_package_share_directory('gowtham_ros')

    # robot_state_publisher (and RViz) from rsp.launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Gazebo with your custom hospital.world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'hospital.world'),
            'gui': 'true'
        }.items()
    )

    # Spawn your robot into Gazebo after a short delay
    spawn = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'my_bot',
                    '-x', '0', '-y', '0', '-z', '0.01'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    return LaunchDescription([
        declare_sim_time,
        rsp,
        gazebo,
        spawn,
        # NOTE: RViz and teleop_twist_keyboard have been removed.
        #       Launch them manually with:
        #         ros2 run rviz2 rviz2 -d <your_rviz_config>
        #         ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ])
