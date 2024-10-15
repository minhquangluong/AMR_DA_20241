import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # params
    params_file = os.path.join(get_package_share_directory('dynamic_transform_publisher'), 'config', 'params.yaml')

    # odom
    dynamic_transform_node = Node(
        package='dynamic_transform_publisher',
        executable='dynamic_transform', 
        name='dynamic_tf',
        output='screen',
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
    # hall
    
    feed_back_node = Node(
        package='dynamic_transform_publisher',
        executable='hall_back',  
        name='feed_back',
        # output='screen',
        parameters=[params_file],  
    )

    return LaunchDescription([
        dynamic_transform_node,
        robot_transform_node,
        feed_back_node,
    ])
