from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

# Main launch file. It should launch all of the necessary manipulator components
def generate_launch_description():
    pkg_share = FindPackageShare("arm_robot")
    moveit_config_pkg = get_package_share_directory('arm_robot_moveit_config')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, "launch", "display.launch.py"
            ])
        )
    )

    motion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, "launch", "motion_controller.launch.py"
            ])
        )
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, "launch", "path_planner.launch.py"
            ])
        )
    )

    return LaunchDescription([
        display,
        moveit_launch,
        motion,
        planner
    ])