import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro # Needed if your URDF is xacro

def generate_launch_description():
    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_your_project = get_package_share_directory('my_turtlebot3_controller') # Verify your package name
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    # For Foxy, URDFs are often in turtlebot3_description, not turtlebot3_gazebo
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')


    # Your Gazebo world file
    world_file_path = os.path.join(pkg_your_project, 'worlds', 'my_trash_world.world')

    # Launch Configurations
    model = LaunchConfiguration('model')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name') # Define robot name

    # Declare launch arguments
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model type (burger, waffle, waffle_pi)')

    declare_x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    declare_y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    declare_z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.0')
    declare_yaw_pose_arg = DeclareLaunchArgument('yaw_pose', default_value='0.0')
    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_robot_name_arg = DeclareLaunchArgument('robot_name', default_value='turtlebot3') # Default robot name


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
    # This launch file IS present in turtlebot3_gazebo/launch for foxy-devel
    rsp_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'model': model}.items()
    )

    # --- Corrected Spawning Logic for Foxy ---
    # Get the URDF file path
    # Note: In Foxy, the URDFs are typically in the 'turtlebot3_description' package
    urdf_file_name_subst = PythonExpression(["'turtlebot3_', model, '.urdf'"])
    urdf_path = PythonExpression([
        "'", os.path.join(pkg_turtlebot3_description, 'urdf'), "/'",
        " + ", urdf_file_name_subst,
    ])
    # If your URDF is a .urdf.xacro file, you'd need to process it first.
    # For standard .urdf files provided by turtlebot3_description, this is okay.

    # Spawn TurtleBot3
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name, # Use the robot_name LaunchConfiguration
            '-file', urdf_path,    # This will be the evaluated path to the URDF
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose         # Note: spawn_entity.py uses -Y for yaw
        ],
        output='screen'
    )
    # --- END Corrected Spawning Logic ---

    return LaunchDescription([
        declare_model_arg,
        declare_x_pose_arg,
        declare_y_pose_arg,
        declare_z_pose_arg,
        declare_yaw_pose_arg,
        declare_use_sim_time_arg,
        declare_robot_name_arg, # Declare the robot_name argument

        gzserver_cmd,
        gzclient_cmd,
        rsp_cmd,
        spawn_entity_cmd, # Add the direct spawn command
    ])