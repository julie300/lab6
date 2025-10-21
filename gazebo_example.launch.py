import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_my_world = get_package_share_directory('') # TODO: your package name
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='') # TODO: choose spawn position
    y_pose = LaunchConfiguration('y_pose', default='') # TODO: choose spawn position
    
    # World file path
    world = os.path.join(pkg_my_world, 'worlds', '.world') # TODO: fill the name of your cutom world
    
    # Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher - this will publish the TF tree from the URDF
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf = os.path.join(pkg_turtlebot3_description, 'urdf', urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        robot_desc = robot_desc.replace('${namespace}', '')

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'frame_prefix': ''  
        }]
    )

    # Spawn TurtleBot3 - this will spawn the robot model and the associated plugins
    turtlebot3_model_path = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle_pi', 'model.sdf') # TODO: check the model used
    
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-file', turtlebot3_model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1'
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='', description='Initial x position of the robot')) # TODO: choose spawn position
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='', description='Initial y position of the robot')) # TODO: choose spawn position

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd) # Remove the extra comma here

    return ld
