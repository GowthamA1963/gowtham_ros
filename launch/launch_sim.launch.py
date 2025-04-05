import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'gowtham_ros'  # Your package name

    # Include robot_state_publisher launch file with sim time enabled
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include Gazebo launch file (by default launches with GUI)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # Delay the spawn_entity node to ensure Gazebo and the robot_state_publisher are fully up
    spawn_entity = TimerAction(
        period=5.0,  # Delay in seconds (adjust if needed)
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
    ])
