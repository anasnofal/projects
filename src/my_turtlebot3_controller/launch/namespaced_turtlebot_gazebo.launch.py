import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Get the TURTLEBOT3_MODEL environment variable, default to 'burger'
    # This should be 'burger', 'waffle', or 'waffle_pi'
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_tb3', # You can change the default namespace here
        description='Namespace for the TurtleBot3'
    )
    # This is the Substitution object that will carry the value of the 'robot_name' launch argument
    robot_name_lc = LaunchConfiguration('robot_name')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define paths to standard ROS 2 packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo') # Used for worlds

    # --- World File Definition ---
    # This constructs a path like: 'turtlebot3_worlds/burger.model'
    # based on the TURTLEBOT3_MODEL environment variable.
    world_model_relative_path = PathJoinSubstitution([
        TextSubstitution(text='turtlebot3_worlds/'), # The subdirectory
        TextSubstitution(text=TURTLEBOT3_MODEL),     # e.g., 'burger', 'waffle', or 'waffle_pi'
        TextSubstitution(text='.model')              # The file extension
    ])

    world_path = PathJoinSubstitution([
        pkg_turtlebot3_gazebo, # Should resolve to /opt/ros/foxy/share/turtlebot3_gazebo (or your workspace)
        'worlds',              # The main 'worlds' directory
        world_model_relative_path # The 'turtlebot3_worlds/MODEL.model' part
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
    # URDF file path for the specified TURTLEBOT3_MODEL
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = PathJoinSubstitution([
        pkg_turtlebot3_description,
        'urdf',
        urdf_file_name
    ])

    # Construct parts for frame_prefix. This list of substitutions will be joined.
    # Results in '<robot_name_lc_value>/'
    frame_prefix_parts = [robot_name_lc, TextSubstitution(text='/')]

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',         # Node name
        namespace=robot_name_lc,             # Namespace for the node
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PythonExpression(["open('", urdf_path, "').read()"]), # Reads URDF content
            'frame_prefix': frame_prefix_parts # Applies the namespace to TF frames
        }]
    )

    # --- Spawn Robot Entity ---
    # Construct parts for the robot_description topic name.
    # Results in '/<robot_name_lc_value>/robot_description'
    robot_description_topic_parts = [TextSubstitution(text='/'), robot_name_lc, TextSubstitution(text='/robot_description')]

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', robot_description_topic_parts, # Topic where URDF is published
            '-entity', robot_name_lc,                # Name of the model in Gazebo
            '-robot_namespace', robot_name_lc,       # Namespace for ROS topics from Gazebo plugins
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',                             # Small offset from the ground
            '-Y', '0.0'                              # Initial yaw
        ],
        output='screen'
    )

    # Log messages for debugging
    log_world_path = LogInfo(msg=['Attempting to load world: ', world_path])
    log_urdf_path = LogInfo(msg=['Loading URDF from: ', urdf_path, ' for model: ', TURTLEBOT3_MODEL])
    log_robot_description_topic_msg_parts = ['Robot description topic: '] + robot_description_topic_parts
    log_robot_description_topic = LogInfo(msg=log_robot_description_topic_msg_parts)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        robot_name_arg, # Declare the robot_name launch argument

        # Log actions for debugging
        log_world_path,
        log_urdf_path,
        log_robot_description_topic,

        # Gazebo
        gzserver_cmd,
        gzclient_cmd,

        # Robot
        robot_state_publisher_node,
        spawn_entity_cmd,
    ])