from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_robot',
            executable='motion_controller_node',
            output='screen'
        )
    ])