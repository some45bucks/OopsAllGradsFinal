from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    joy_node = launch_ros.actions.Node(
      package='joy_linux',
      executable='joy_linux_node',
      name='joy_node',
    )
    
    controller_node = launch_ros.actions.Node(
      package='final_project',
      executable='wireless_controller',
      name='wireless_controller',
      output ='screen'
    )

    ld = LaunchDescription()

    ld.add_action(joy_node)
    ld.add_action(controller_node)

    return ld
