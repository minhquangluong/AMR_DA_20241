import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # params
    params_file = os.path.join(get_package_share_directory('handle_pkg'), 'config', 'params.yaml')

    # handle
    handle_node = Node(
        package='handle_pkg',
        executable='robot_handle', 
        name='robot_handle',
        output='screen',
        parameters=[params_file], 
    )
     

    return LaunchDescription([
        handle_node,
    
    ])
