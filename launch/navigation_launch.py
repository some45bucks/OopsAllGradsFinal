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

    nav_pkg = 'wheeltec_nav2'
    nav_file_name = 'wheeltec_nav2.launch.py'

    nav_dir = get_package_share_directory(nav_pkg)
    nav_launch_file_path = os.path.join(nav_dir, 'launch', nav_file_name)

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'akmcar': 'false'}.items(),
    )

    navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_file_path),
    )

    ld = LaunchDescription()

    ld.add_action(wheeltec_robot)
    ld.add_action(navigation)
    
    return ld

