import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the TURTLEBOT3_MODEL environment variable, default to 'burger'
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_tb3',
        description='Namespace for the TurtleBot3'
    )
    # This is the Substitution object that will carry the value
    robot_name_lc = LaunchConfiguration('robot_name')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define paths to standard ROS 2 packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # --- World File Definition ---
    world_file_to_load = 'turtlebot3_world.world'

    world_path = PathJoinSubstitution([
        pkg_turtlebot3_gazebo,
        'worlds',
        world_file_to_load
    ])

    # --- Gazebo Server ---
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # --- Gazebo Client ---
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        )
    )

    # --- Robot State Publisher ---
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = PathJoinSubstitution([
        pkg_turtlebot3_description,
        'urdf',
        urdf_file_name
    ])

    # Construct parts for frame_prefix. This list of substitutions will be joined.
    frame_prefix_parts = [robot_name_lc, TextSubstitution(text='/')]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_lc, # Use the LaunchConfiguration for namespacing the node
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PythonExpression(["open('", urdf_path, "').read()"]), # PythonExpression is fine here for reading file
            'frame_prefix': frame_prefix_parts # Pass the list of substitutions
        }]
    )

    # --- Spawn Robot Entity ---
    # Construct parts for the robot_description topic name
    # This list of substitutions will be joined by the launch system.
    robot_description_topic_parts = [TextSubstitution(text='/'), robot_name_lc, TextSubstitution(text='/robot_description')]

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', robot_description_topic_parts, # Pass the list of substitutions
            '-entity', robot_name_lc,                # Use LaunchConfiguration for entity name
            '-robot_namespace', robot_name_lc,       # Use LaunchConfiguration for plugin namespace
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Corrected Log messages for debugging
    # LogInfo can take a list of strings and Substitution objects
    log_world_path = LogInfo(msg=['Attempting to load world: ', world_path])
    log_urdf_path = LogInfo(msg=['Loading URDF from: ', urdf_path, ' for model: ', TURTLEBOT3_MODEL])
    # For log_robot_description_topic, we can build the message list
    log_robot_description_topic_msg_parts = ['Robot description topic: '] + robot_description_topic_parts
    log_robot_description_topic = LogInfo(msg=log_robot_description_topic_msg_parts)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        robot_name_arg, # Declare the launch argument

        log_world_path,
        log_urdf_path,
        log_robot_description_topic,

        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_cmd,
    ])