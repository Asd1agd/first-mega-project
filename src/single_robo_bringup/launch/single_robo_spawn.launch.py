from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    single_robo_discription_pkg = FindPackageShare('single_robo_discription')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo server-only (-s)'
    )

    # 1. Launch Gazebo spawn (forward headless)
    gazebo_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                single_robo_discription_pkg,
                'launch',
                'spawn_gazebo.launch.py'
            ])
        ]),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items()
    )

    ld = LaunchDescription()
    ld.add_action(headless_arg)
    ld.add_action(gazebo_spawn_launch)
    
    return ld