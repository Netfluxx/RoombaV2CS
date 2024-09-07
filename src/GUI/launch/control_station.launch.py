#LAUNCH FILE FOR SLAM AND MANUAL CONTROL

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('GUI'), 'config', 'joystick_params.yaml')

    #launch the gui nodoe and the joystick launch file in the controller_input package
    
    gui_control_station = Node(
        package='GUI',
        executable='gui',
        name='gui',
        output='screen',
        parameters=[]
    )

    #launch the joystick stuff
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/joystick_cmd_vel')]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        gui_control_station
    ])

