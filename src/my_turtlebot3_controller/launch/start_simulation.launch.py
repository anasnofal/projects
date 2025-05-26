import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_smart_waste_project = get_package_share_directory('smart_waste_project') # Your package name
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Your Gazebo world file
    # Make sure 'my_trash_world.world' is in 'smart_waste_project/worlds/'
    world_file_path = os.path.join(pkg_smart_waste_project, 'worlds', 'my_trash_world.world')

    # TurtleBot3 model: 'burger', 'waffle', 'waffle_pi'
    model = LaunchConfiguration('model', default_value='burger')
    # Spawn pose for TurtleBot3
    x_pose = LaunchConfiguration('x_pose', default_value='0.0')
    y_pose = LaunchConfiguration('y_pose', default_value='0.0')
    z_pose = LaunchConfiguration('z_pose', default_value='0.0')
    yaw_pose = LaunchConfiguration('yaw_pose', default_value='0.0')


    # Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path, 'verbose': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher for TurtleBot3
    # This loads the URDF model of the TurtleBot3
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 'model': model}.items()
    )

    # Spawn TurtleBot3
    spawn_turtlebot3_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose,
            'model': model,
            'robot_name': 'turtlebot3' # Or any name you prefer
        }.items()
    )

    # Declare launch arguments (optional, allows changing model/pose from command line)
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type (burger, waffle, waffle_pi)')

    declare_x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.0')
    declare_yaw_pose_arg = DeclareLaunchArgument('yaw_pose', default_value='0.0')


    return LaunchDescription([
        declare_model_arg,
        declare_x_pose_arg,
        declare_y_pose_arg,
        declare_z_pose_arg,
        declare_yaw_pose_arg,
        gzserver_cmd,
        gzclient_cmd,
        rsp_cmd,
        spawn_turtlebot3_cmd,
    ])