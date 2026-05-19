"""Launch apriltag3_ros with the example parameter overrides."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('apriltag3_ros')
    default_params = os.path.join(pkg_share, 'config', 'apriltag3_ros.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the YAML parameter overrides for the apriltag3_ros node.',
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='image_rect',
        description='Image topic to subscribe to.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera_info',
        description='CameraInfo topic to subscribe to.',
    )

    node = Node(
        package='apriltag3_ros',
        executable='apriltag3_ros',
        name='apriltag3_ros',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'image_topic': LaunchConfiguration('image_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_file_arg,
        image_topic_arg,
        camera_info_topic_arg,
        node,
    ])
