from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pgv_scanner',
            executable='pgv_driver',
            name='pgv_driver',
            output='screen'
        )
        
        
        
        
        
    ])
