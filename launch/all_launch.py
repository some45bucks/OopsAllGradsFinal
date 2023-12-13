import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path

def generate_launch_description():
    current_package_launch_dir = os.path.dirname(os.path.abspath(__file__))

    mapping = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(current_package_launch_dir, 'mapping_launch.py'))
    )

    detection = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(current_package_launch_dir, 'person_detection_launch.py'))
    )

    ld = LaunchDescription()

    ld.add_action(mapping)
    ld.add_action(detection)
    
    return ld

