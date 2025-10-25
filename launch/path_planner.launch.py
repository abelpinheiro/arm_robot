from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_robot',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            emulate_tty=True
        )
    ])