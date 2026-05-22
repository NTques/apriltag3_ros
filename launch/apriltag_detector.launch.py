from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('apriltag3_ros')
    default_params = PathJoinSubstitution(
        [pkg_share, 'config', 'apriltag_detector.yaml'])

    params_file = LaunchConfiguration('params_file')
    image_topic = LaunchConfiguration('image_topic')
    container_name = LaunchConfiguration('container_name')
    use_intra_process = LaunchConfiguration('use_intra_process')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Absolute path to the AprilTagDetector parameters YAML.'),
        DeclareLaunchArgument(
            'image_topic',
            default_value='image_rect',
            description=(
                'Remap target for the input image topic. The matching '
                'camera_info topic is taken as its sibling automatically.')),
        DeclareLaunchArgument(
            'container_name',
            default_value='apriltag_container',
            description='Composable node container name.'),
        DeclareLaunchArgument(
            'use_intra_process',
            default_value='true',
            description='Enable intra-process comms inside the container.'),

        ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='apriltag3_ros',
                    plugin='apriltag3_ros::AprilTagDetector',
                    name='apriltag_detector',
                    parameters=[params_file],
                    remappings=[
                        ('image_rect', image_topic),
                    ],
                    extra_arguments=[
                        {'use_intra_process_comms': use_intra_process},
                    ],
                ),
            ],
            output='screen',
        ),
    ])
