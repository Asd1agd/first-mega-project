from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to various package directories
    single_robo_discription_pkg = FindPackageShare('single_robo_discription')
    single_robo_odometry_pkg = FindPackageShare('single_robo_odometry')
    
    # Path to parameter file
    param_file_path = PathJoinSubstitution([
        single_robo_discription_pkg,
        'config',
        'ps4_config.yaml'
    ])

    rviz_config_path = PathJoinSubstitution([
        single_robo_discription_pkg,
        'config',
        'rviz_config.rviz'
    ])
    
    # 1. Launch Gazebo spawn
    gazebo_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                single_robo_discription_pkg,
                'launch',
                'spawn_gazebo.launch.py'
            ])
        ])
    )
    
    # 2. odom_tf_pub node
    odom_tf_pub_node = Node(
        package='single_robo_odometry',
        executable='odom_tf_pub',
        name='odom_tf_pub',
        output='screen'
    )
    
    # 3. odometry_calculation node
    odometry_calculation_node = Node(
        package='single_robo_odometry',
        executable='odometry_calculation',
        name='odometry_calculation',
        output='screen'
    )
    
    # 4. odom_tf_pub_w_error node
    odom_tf_pub_w_error_node = Node(
        package='single_robo_odometry',
        executable='odom_tf_pub_w_error',
        name='odom_tf_pub_w_error',
        output='screen'
    )
    
    # 5. odometry_error node
    odometry_error_node = Node(
        package='single_robo_odometry',
        executable='odometry_error',
        name='odometry_error',
        output='screen'
    )
    
    # 6. joy_node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    # 7. teleop_twist_joy node with parameter file
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[param_file_path]
    )
    
    # 8. rviz2 - using Node instead of ExecuteProcess
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],  # Optional: load specific config
        output='screen'
    )

    # Delayed start for rviz2 to ensure other nodes are running
    delayed_rviz2 = TimerAction(
        period=10.0,  # 5 second delay
        actions=[rviz2_node]
    )

    # Delayed start for rviz2 to ensure other nodes are running
    delayed_joy_node = TimerAction(
        period=2.0,  # 5 second delay
        actions=[joy_node]
    )

    # Delayed start for rviz2 to ensure other nodes are running
    delayed_teleop_node = TimerAction(
        period=10.0,  # 5 second delay
        actions=[teleop_node]
    )    


    # Create launch description
    ld = LaunchDescription()
    
    # Add gazebo spawn first
    ld.add_action(gazebo_spawn_launch)

    # Add joystick control nodes
    ld.add_action(joy_node)
    ld.add_action(delayed_teleop_node)
    
    # Add odometry nodes
     # Phase 2: Wait for Gazebo, then start robot nodes
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            odom_tf_pub_node,
            odometry_calculation_node,
            odom_tf_pub_w_error_node,
            odometry_error_node
        ]
    ))
    
    # Add delayed rviz2
    ld.add_action(delayed_rviz2)
    
    return ld

