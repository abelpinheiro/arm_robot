from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

# Main launch file. It should launch all of the necessary manipulator components
def generate_launch_description():
    pkg_share = FindPackageShare("arm_robot")

    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share, "launch", "display.launch.py"
            ])
        )
    )

    return LaunchDescription([
        display
    ])