import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from pathlib import Path

def generate_launch_description():

    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'akmcar': 'false'}.items(),
    )

    current_package_launch_dir = os.path.dirname(os.path.abspath(__file__))

    gamepad = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(current_package_launch_dir, 'gamepad_nodes_launch.py'))
    )

    ld = LaunchDescription()

    ld.add_action(wheeltec_robot)
    ld.add_action(gamepad)
    
    return ld

