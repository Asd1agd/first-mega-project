from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    single_robo_bringup_pkg = get_package_share_directory('single_robo_bringup')

    # 1. Launch Gazebo spawn
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                single_robo_bringup_pkg,
                'launch',
                'single_robo_spawn.launch.py'
            ])
        ])
    )

    # 2. Launch navigation and mapping
    nav_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                single_robo_bringup_pkg,
                'launch',
                'single_robo_nav_map.launch.py'
            ])
        ])
    )

    # 2. Launch navigation and mapping
    nav2_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                single_robo_bringup_pkg,
                'launch',
                'single_robo_navigation.launch.py'
            ])
        ])
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(spawn_launch)
    ld.add_action(nav_map_launch)
    ld.add_action(nav2_stack_launch)

    return ld