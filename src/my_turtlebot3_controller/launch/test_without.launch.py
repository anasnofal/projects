import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from launch_ros.actions import Node

def generate_launch_description():
    my_controller_pkg_dir = get_package_share_directory('my_turtlebot3_controller')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_pkg_dir = get_package_share_directory('slam_toolbox')

    turtlebot3_gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    slam_params_file = LaunchConfiguration('slam_params_file',
        default=os.path.join(my_controller_pkg_dir, 'config', 'slam_simulation.yaml'))
        
    rviz_config_file = LaunchConfiguration('rviz_config',
        default=os.path.join(nav2_bringup_pkg_dir, 'rviz', 'nav2_default_view.rviz'))

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_pkg_dir, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 2. Launch SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_pkg_dir, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params_file
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': ''  
            }.items(),
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])