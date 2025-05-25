import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
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
    robot_name = LaunchConfiguration('robot_name')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define paths to standard ROS 2 packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo') # Used for worlds

    # --- World File Definition ---
    # Explicitly choose a world file that you know exists.
    # These are standard worlds provided by turtlebot3_gazebo.
    # world_file_to_load = 'empty_world.world'
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
    # URDF file path for the specified TURTLEBOT3_MODEL
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = PathJoinSubstitution([
        pkg_turtlebot3_description,
        'urdf',
        urdf_file_name
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', # Node name
        namespace=robot_name,         # Namespace for the node
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PythonExpression(["open('", urdf_path, "').read()"]),
            # To correctly namespace TF frames, use frame_prefix.
            # The topics /tf and /tf_static will be automatically namespaced
            # if the node is in a namespace.
            # For TF frames themselves, 'frame_prefix' is needed.
            'frame_prefix': PythonExpression(["'", robot_name, "/'"])
        }]
        # Remapping for /tf and /tf_static is usually handled by the node's namespace.
        # If explicit remapping is needed:
        # remappings=[
        #     ('/tf', 'tf'),
        #     ('/tf_static', 'tf_static')
        # ]
    )

    # --- Spawn Robot Entity ---
    # The robot_description topic will be namespaced as /<robot_name>/robot_description
    # due to the robot_state_publisher_node being in a namespace.
    robot_description_topic_name = PythonExpression(["'/', robot_name, '/robot_description'"])

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', robot_description_topic_name,
            '-entity', robot_name,                # Name of the model in Gazebo
            '-robot_namespace', robot_name,       # Namespace for ROS topics from Gazebo plugins
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',                          # Small offset
            '-Y', '0.0'                           # Initial yaw
        ],
        output='screen'
    )

    # Log messages for debugging
    log_world_path = LogInfo(msg=PythonExpression(["'Attempting to load world: ', '", world_path, "'"]))
    log_urdf_path = LogInfo(msg=PythonExpression(["'Loading URDF from: ', '", urdf_path, "' for model: ", TURTLEBOT3_MODEL, "'"]))
    log_robot_description_topic = LogInfo(msg=PythonExpression(["'Robot description topic: ', '", robot_description_topic_name, "'"]))


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        robot_name_arg,

        log_world_path,
        log_urdf_path,
        log_robot_description_topic,

        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node,
        spawn_entity_cmd,
    ])