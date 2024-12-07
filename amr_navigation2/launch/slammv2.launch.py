from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
 
    slam_config = '/home/quangluong/AMR_DA_ws/src/amr_navigation2/config/mapper_params_online_async.yaml'
    
   
    ekf_config = '/home/quangluong/AMR_DA_ws/src/amr_navigation2/config/ekf.yaml'
    
    return LaunchDescription([
        # Robot Localization 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        ),
        
        # SLAM Toolbox 
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                slam_config
            ],
        )
    ])