from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='pygame_node',
            name='pygame_node',
            output='screen'
        ),
        Node(
            package='robot_control',
            executable='task_planning',
            name='task_planning_node',
            output='screen'
        ),
    ])
