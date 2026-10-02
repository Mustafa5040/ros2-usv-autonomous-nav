import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ws_path = '/root/ida_ws'
    
    launch_vision_path = os.path.join(ws_path, 'launch', 'vision.launch.py')
    launch_nav_path    = os.path.join(ws_path, 'launch', 'navigation.launch.py')
    
    script_state_machine = os.path.join(ws_path, 'state_machine.py')

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_vision_path)
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_nav_path)
    )


    state_machine_node = ExecuteProcess(
        cmd=['python3', script_state_machine],
        output='screen',
        name='state_machine'
    )

    return LaunchDescription([
        LogInfo(msg='=========================================='),
        LogInfo(msg='   IDA MISSION AUTONOMY STARTING     '),
        LogInfo(msg='=========================================='),

        vision_launch,
        nav_launch,

        TimerAction(
            period=10.0,
            actions=[
                LogInfo(msg='>> Starting State Machine...'),
                state_machine_node
            ]
        )
    ])