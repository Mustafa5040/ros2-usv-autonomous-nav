import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ws_path = '/root/ida_ws'

    script_oak = os.path.join(ws_path, 'vision/oak_node.py')

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[('cloud_in', '/oak/rgbd/points'),
                    ('scan', '/stereo/scan')],
        parameters=[{
            'target_frame': 'base_link',
            'min_height': 0.05,
            'max_height': 0.5,
            'range_max': 15.0,
        }]
    )   

    oak_node = ExecuteProcess(
        cmd=['python3', script_oak],
        output='screen',
        name='oak_node'
    )

    return LaunchDescription(  
        [
        LogInfo(msg='VISION DEBUG LAUNCHING...'),

        TimerAction(
            period=5.0,
            actions=[oak_node]
        ),
        #pointcloud_to_laserscan_node,

    ])