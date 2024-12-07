import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_amr_description = get_package_share_directory('amr_description')
    
    # World file
    world_file_name = 'myworld.world'
    world = os.path.join(pkg_amr_description, 'world', world_file_name)
    
    # URDF file
    urdf_file_path = os.path.join(pkg_amr_description, 'urdf', 'robot.urdf')
    
    # Read URDF content
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_content,
                'use_sim_time': use_sim_time,
                'publish_frequency': 30.0,
            }]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'dd_robot',
                '-file', urdf_file_path,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1'
            ]
        ),
    ])