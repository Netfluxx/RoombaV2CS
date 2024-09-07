from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('controller_input'), 'config', 'joystick_params.yaml')

    # Define the QoS profile with BEST EFFORT reliability
    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,  # Keep the last 10 messages
        reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Use BEST EFFORT reliability
        durability=QoSDurabilityPolicy.VOLATILE
    )


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
        teleop_node
    ])
