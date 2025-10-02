from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Guided mode switcher
        Node(
            package='gate_detector',
            executable='guided_mode',
            name='guided_mode',
            output='screen'
        ),

        # 2. Depth controller
        Node(
            package='gate_detector',
            executable='depth_controller',
            name='depth_controller',
            output='screen'
        ),

        # 3. Gate aligner
        Node(
            package='gate_detector',
            executable='gate_aligner',
            name='gate_aligner',
            output='screen'
        ),

        # 4. Gate committer
        Node(
            package='gate_detector',
            executable='gate_committer',
            name='gate_committer',
            output='screen'
        ),
    ])
