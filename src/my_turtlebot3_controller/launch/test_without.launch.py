import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')
    my_controller_pkg_dir = get_package_share_directory('my_turtlebot3_controller')
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    slam_params_file = LaunchConfiguration('slam_params_file',
        default=os.path.join(my_controller_pkg_dir, 'config', 'slam_config_dt.yaml'))
        
    rviz_config_file = LaunchConfiguration('rviz_config',
        default=os.path.join(nav2_bringup_pkg_dir, 'rviz', 'nav2_default_view.rviz'))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file,
        description='Full path to the SLAM parameters file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='Full path to the RViz configuration file to use')

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_pkg_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items(),
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # This is the new delayed action for Nav2
    delayed_nav2_bringup_cmd = TimerAction(
        period=5.0,  # Wait for 5 seconds before executing the action
        actions=[
            LogInfo(msg='Timer expired. Launching Nav2...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_pkg_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': slam_params_file
                }.items(),
            )
        ]
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    
    # Launch Gazebo, SLAM, and RViz immediately
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_rviz_cmd)
    
    # Add the delayed Nav2 launch
    ld.add_action(LogInfo(msg='Waiting 5 seconds before launching Nav2 to allow other nodes to start...'))
    ld.add_action(delayed_nav2_bringup_cmd)

    return ld