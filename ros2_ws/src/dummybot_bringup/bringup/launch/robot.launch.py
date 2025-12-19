from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    start_rviz = LaunchConfiguration("start_rviz")

    # Path to base launch file
    base_launch = PathJoinSubstitution([
        FindPackageShare("dummybot_bringup"),
        "bringup",
        "launch",
        "base.launch.py"
    ])

    # Include base launch
    include_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([base_launch]),
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "start_rviz": start_rviz,
        }.items()
    )

    # Future: aici poți adăuga alte launch-uri (sensors, navigation, etc.)
    # include_sensors_launch = IncludeLaunchDescription(...)
    # include_navigation_launch = IncludeLaunchDescription(...)

    return LaunchDescription(declared_arguments + [
        include_base_launch,
        # include_sensors_launch,
        # include_navigation_launch,
    ])