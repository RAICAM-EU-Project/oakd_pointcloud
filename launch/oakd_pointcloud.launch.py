from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_pointcloud',
            executable='oakd_pointcloud_publisher',
            name='oakd_pointcloud_node',
            output='screen'
        )
    ])
