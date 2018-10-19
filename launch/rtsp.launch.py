import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_file = os.path.join(get_package_share_directory('ros2_videostreamer'),
                        'config', 'config.yaml')
    arguments = "__params:="+param_file

    return LaunchDescription([
       Node(package='ros2_videostreamer', node_executable='rtsp_node', node_namespace='ros2_videostreamer', node_name='rtsp_node', arguments=[arguments], output='screen'),
    ])