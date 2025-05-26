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
    # Make sure to replace 'my_turtlebot3_controller' with your actual package name if different
    # The error log shows: /home/anas/projects/install/my_turtlebot3_controller/share/my_turtlebot3_controller/launch/start_simulation.launch.py
    # So, your package name seems to be 'my_turtlebot3_controller'
    pkg_your_project = get_package_share_directory('my_turtlebot3_controller')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Your Gazebo world file
    # Ensure 'my_trash_world.world' is in 'pkg_your_project/worlds/'
    world_file_path = os.path.join(pkg_your_project, 'worlds', 'my_trash_world.world') # Adjusted to use pkg_your_project

    # --- CORRECTED LaunchConfiguration ---
    # TurtleBot3 model: 'burger', 'waffle', 'waffle_pi'
    # For Foxy, LaunchConfiguration only takes the name. Default is set in DeclareLaunchArgument.
    model = LaunchConfiguration('model')
    # Spawn pose for TurtleBot3
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    # --- END CORRECTION ---

    # Declare launch arguments
    # These set the default values which LaunchConfiguration will then use.
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger', # Default value is set here
        description='TurtleBot3 model type (burger, waffle, waffle_pi)')

    declare_x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0', # Default value is set here
        description='Initial x pose of the robot')

    declare_y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0', # Default value is set here
        description='Initial y pose of the robot')

    declare_z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0', # Default value is set here
        description='Initial z pose of the robot')

    declare_yaw_pose_arg = DeclareLaunchArgument(
        'yaw_pose',
        default_value='0.0', # Default value is set here
        description='Initial yaw pose of the robot')

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
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true', 'model': model}.items() # 'model' here refers to the LaunchConfiguration
    )

    # Spawn TurtleBot3
    spawn_turtlebot3_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose, # These refer to the LaunchConfigurations
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw_pose': yaw_pose,
            'model': model,
            'robot_name': 'turtlebot3'
        }.items()
    )

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