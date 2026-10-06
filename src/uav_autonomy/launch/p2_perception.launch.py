"""Launch independent debris tracking and survivor candidate/VLM pipelines."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Launch image-plane debris tracks and separate survivor verification."""
    camera_topic = LaunchConfiguration('camera_topic')
    vlm_endpoint = LaunchConfiguration('vlm_endpoint')
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('vlm_endpoint', default_value=''),
        Node(
            package='uav_autonomy', executable='debris_perception',
            name='debris_perception', output='screen',
            parameters=[{'image_topic': camera_topic}],
        ),
        Node(
            package='uav_autonomy', executable='survivor_perception',
            name='survivor_perception', output='screen',
            parameters=[{'image_topic': camera_topic,
                         'vlm_endpoint': vlm_endpoint}],
        ),
    ])
