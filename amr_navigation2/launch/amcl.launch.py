from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from lifecycle_msgs.msg import Transition
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action

def generate_launch_description():
    # Node declarations
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': '/home/quangluong/AMR_DA_ws/src/amr_navigation2/map/map.yaml'}]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'min_particles': 100},
            {'max_particles': 500},
            {'laser_model_type': 'likelihood_field'}
        ],
        remappings=[
            ('/scan', '/scan')
        ]
    )

    # Configure and activate map_server
    configure_map_server_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_server_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    # Configure and activate AMCL
    configure_amcl_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_amcl_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    # Event handlers for lifecycle transitions
    map_server_configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server_node,
            on_start=[TimerAction(period=2.0, actions=[configure_map_server_event])]
        )
    )

    map_server_activate_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server_node,
            on_start=[TimerAction(period=3.0, actions=[activate_map_server_event])]
        )
    )

    amcl_configure_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=amcl_node,
            on_start=[TimerAction(period=2.0, actions=[configure_amcl_event])]
        )
    )

    amcl_activate_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=amcl_node,
            on_start=[TimerAction(period=3.0, actions=[activate_amcl_event])]
        )
    )

    return LaunchDescription([
        # Lifecycle node handlers
        map_server_configure_handler,
        map_server_activate_handler,
        amcl_configure_handler,
        amcl_activate_handler,
        
        # Nodes
        map_server_node,
        amcl_node,
        
        # Include statements
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dynamic_transform_publisher'),  
                    'launch',
                    'dynamic_transform_publisher_launch.py'  
                ])
            ])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rplidar_ros'),
                    'launch',
                    'rplidar_a1_launch.py'
                ])
            ])
        ),
    ])