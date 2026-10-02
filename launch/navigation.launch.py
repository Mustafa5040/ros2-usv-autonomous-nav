import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    nav2_params_file = '/root/ida_ws/navigation/config/ida_nav2_params.yaml'
    ekf_params_file = '/root/ida_ws/navigation/config/ekf_map.yaml'
    # Cam height: 0.53 meters and tilt down angle: -0.2618 radians
    tf_base_to_camera = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_camera',
    arguments=['--x', '0.41', '--y', '0', '--z', '0.185',
               '--yaw', '0.0', '--pitch', '0', '--roll', '0',
               '--frame-id', 'base_link',
               '--child-frame-id', 'oak_camera_frame']
)
    tf_base_to_ned = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_ned',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '3.1416',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_link_ned']
    )

    tf_camera_to_optical = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_optical',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--yaw', '-1.5707', '--pitch', '0', '--roll', '-1.5707',
                   '--frame-id', 'oak_camera_frame',
                   '--child-frame-id', 'oak_rgb_camera_optical_frame']
    )
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/oak/rgbd/points'),
            ('scan', '/oak/rgbd/scan')
        ],
        parameters=[{
            'target_frame': 'oak_camera_frame',
            'transform_tolerance': 0.01,
            'min_height': -0.05,
            'max_height': 1.0,
            'angle_min': -0.7, 
            'angle_max': 0.7, 
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.4,
            'range_max': 10.0,
            'use_inf': True
        }]
)

    odom_bridge = ExecuteProcess(
        cmd=['python3','/root/ida_ws/navigation/odom_bridge.py'],
        output='screen'
    )
    
    gps_initialpose = ExecuteProcess(
        cmd=['python3','/root/ida_ws/navigation/gps_initialpose.py'],
        output='screen'
    )

    cmd_vel_bridge = ExecuteProcess(
        cmd=['python3', '/root/ida_ws/navigation/cmd_vel_bridge.py'],
        output='screen'
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params_file]
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': True},
            {'node_names': ['controller_server', 
                            'planner_server', 
                            'behavior_server', 
                            'bt_navigator',
                            'velocity_smoother',
                            'waypoint_follower']},
            {'bond_timeout': 10.0}
        ]
    )



    return LaunchDescription([
        tf_base_to_camera,
        tf_camera_to_optical,
        odom_bridge,
        gps_initialpose,
        cmd_vel_bridge,
        tf_base_to_ned,
        #pointcloud_to_laserscan_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        velocity_smoother
    ])
    