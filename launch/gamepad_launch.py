import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')


    mapping_pkg = 'wheeltec_cartographer'
    mapping_file_name = 'cartographer.launch.py'

    mapping_dir = get_package_share_directory(mapping_pkg)
    mapping_launch_file_path = os.path.join(mapping_dir, 'launch', mapping_file_name)

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
            launch_arguments={'akmcar': 'false'}.items(),
    )

    joy_node = launch_ros.actions.Node(
      package='joy_linux',
      executable='joy_linux_node',
      name='joy_node',
    )
    
    controller_node = launch_ros.actions.Node(
      package='final_project',
      executable='wireless_controller',
      name='wireless_controller'
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(external_launch_file_path)
    )

    ld = LaunchDescription()

    ld.add_action(wheeltec_robot)
    ld.add_action(joy_node)
    ld.add_action(controller_node)
    ld.add_action(mapping)
    
    return ld

