import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # params
    params_file = os.path.join(get_package_share_directory('dynamic_transform_publisher'), 'config', 'params.yaml')

    odom_node = Node(
        package='dynamic_transform_publisher',
        executable='odom_back',  
        name='robot_odom',
        # output='screen',
        parameters=[params_file],  
    )

     # robot
    robot_transform_node = Node(
        package='dynamic_transform_publisher',
        executable='robot_transform',  
        name='robot_tf',
        output='screen',
        parameters=[params_file],  
    )
    
    
    
    return LaunchDescription([
        odom_node,
        robot_transform_node,
        
    ])
