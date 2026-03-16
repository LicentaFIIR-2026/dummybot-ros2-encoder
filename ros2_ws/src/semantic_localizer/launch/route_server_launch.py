"""
Route Server Launch — SAIM Xplorer Semantic Navigation
Pornește Route Server standalone, separat de stack-ul Nav2 existent.

Utilizare:
  ros2 launch semantic_localizer route_server_launch.py

Cu parametri custom:
  ros2 launch semantic_localizer route_server_launch.py \
    graph_filepath:=/path/to/custom_graph.geojson
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.expanduser('~'), 'saim_xplorer', 'src',
            'semantic_localizer', 'config', 'route_server_params.yaml'),
        description='Path to Route Server params YAML')

    graph_file_arg = DeclareLaunchArgument(
        'graph_filepath',
        default_value=os.path.join(
            os.path.expanduser('~'), 'saim_xplorer', 'maps',
            'route_graph_fiir_nav2.geojson'),
        description='Path to semantic-annotated GeoJSON route graph')

    # --- Route Server lifecycle node ---
    route_server_node = LifecycleNode(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'graph_filepath': LaunchConfiguration('graph_filepath')}
        ],
        remappings=[
            # Evită conflictul cu planner_server care publică pe /plan
            ('plan', 'route_plan'),
        ],
    )

    # Auto-configure la pornire
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(route_server_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate după configure
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=route_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(route_server_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        params_file_arg,
        graph_file_arg,
        route_server_node,
        activate_event,
        configure_event,
    ])
