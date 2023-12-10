from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from pathlib import Path

def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_camera.launch.py')),
            launch_arguments={'akmcar': 'false'}.items(),
    )
    return LaunchDescription([
        wheeltec_robot,
        Node(
            package='final_project',  # Replace with the actual name of your ROS 2 package
            executable='person_detection',  # Replace with the actual name of your node script
            name='person_detection',
            emulate_tty=True,
            output='screen',
        ),
    ])
