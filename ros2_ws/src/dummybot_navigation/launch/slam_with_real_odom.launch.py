from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dummybot_navigation = get_package_share_directory('dummybot_navigation')
    pkg_dummybot_description = get_package_share_directory('dummybot_description')
    
    # Parametri
    slam_params_file = LaunchConfiguration('slam_params_file')
    urdf_file = os.path.join(pkg_dummybot_description, 'urdf', 'dummybot_base.urdf')
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dummybot_navigation, 'config', 'slam_toolbox_params.yaml'),
        description='Full path to SLAM parameters'
    )
    
    # 1. Hardware Interface - Odometrie reală
    hardware_interface_node = Node(
        package='dummybot_control',
        executable='esp32_hardware_interface',
        name='esp32_hardware_interface',
        parameters=[
            {'serial_port': '/dev/ttyUSB0'},
            {'baud_rate': 115200},
            {'wheel_radius': 0.0325},
            {'wheel_separation': 0.3863},
            {'ticks_per_revolution': 1760},
            {'update_rate': 30.0}
        ],
        output='screen'
    )
    
    # 2. Robot State Publisher (URDF)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
            'publish_frequency': 30.0  # ADAUGĂ ASTA
        }]
    )

    # Joint State Publisher (pentru roți)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # 3. LiDAR Node
    lidar_node = LifecycleNode(
        package='ldlidar_node',
        executable='ldlidar_node',
        name='ldlidar_publisher',
        output='screen',
        namespace='',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'ldlidar_link'},
            {'port_name': '/dev/ttyUSB1'},
            {'serial_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            {'angle_crop_min': 0.0},
            {'angle_crop_max': 0.0}
        ]
    )
    
    # 4. SLAM Toolbox
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
        namespace=''
    )
    
    # Event handlers pentru lifecycle nodes
    configure_lidar_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    activate_lidar_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lidar_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    configure_slam_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    
    activate_slam_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )
    
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_slam_params_file_cmd)
    
    # Add nodes
    ld.add_action(hardware_interface_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(lidar_node)
    ld.add_action(slam_toolbox_node)
    
    # Add lifecycle events
    ld.add_action(configure_lidar_event)
    ld.add_action(activate_lidar_event)
    ld.add_action(configure_slam_event)
    ld.add_action(activate_slam_event)

    ld.add_action(joint_state_publisher_node)
    
    return ld