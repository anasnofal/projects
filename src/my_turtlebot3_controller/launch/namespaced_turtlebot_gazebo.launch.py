import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Ensure TURTLEBOT3_MODEL is set, e.g., 'burger'
TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

def generate_launch_description():
    # 0. Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_tb3', # This will be the namespace
        description='Namespace for the TurtleBot3 topics'
    )
    robot_name = LaunchConfiguration('robot_name')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to a world file (you can choose any)
    world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model' # This is actually a model, not a world
    # A better default might be an actual .world file from turtlebot3_gazebo or gazebo_ros
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world' # Example actual world file
    )
    
    # Paths to other necessary launch files
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # 1. Launch Gazebo Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, 'verbose': 'true'}.items()
    )

    # 2. Launch Gazebo Client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 3. Robot State Publisher
    # This uses the original, unedited turtlebot3_description package
    # It will publish the robot_description topic in the global namespace by default
    # or you can push it into a namespace if you prefer, though spawn_entity can
    # also find it globally if not namespaced here.
    robot_state_publisher_launch = os.path.join(
        pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py'
    )
    
    # We need to ensure robot_state_publisher publishes robot_description under the namespace
    # if spawn_entity is also namespaced and looking for it there.
    # Or, publish robot_description globally and have spawn_entity find it.
    # For simplicity, let robot_state_publisher operate globally or under its own node name,
    # and spawn_entity will find the /robot_description topic.
    # Alternatively, to ensure clean namespacing for robot_description topic itself:
    
    # Get the URDF path
    urdf_file_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name, # Namespace for the node
        name='robot_state_publisher', # Name of the node
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': PythonExpression(["open('", urdf_file_path, "').read()"])
        }],
        # Remap the /tf and /tf_static topics to be under the namespace
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # 4. Spawn Robot Entity into Gazebo
    # This is the crucial part for namespacing Gazebo plugin topics.
    # The `-robot_namespace` argument is key.
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', PythonExpression(["'/",robot_name,"/robot_description'"]), # Topic where URDF is published by namespaced RSP
            '-entity', robot_name,                   # Name of the model in Gazebo
            '-robot_namespace', robot_name,          # Namespace for ROS topics & services from Gazebo plugins
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1', # Small offset to avoid collision with ground plane
            '-Y', '0.0'  # Initial yaw
        ],
        output='screen'
        # The spawn_entity node itself can also be namespaced if desired, but it's less critical
        # namespace=robot_name
    )

    return LaunchDescription([
        robot_name_arg,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_node, # Use the namespaced RSP node
        spawn_entity_cmd,
    ])

