from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sequence',
            default_value='0',
            description='Sequence to publish (0 or 1)'
        ),
        Node(
            package='customdata_publisher',
            executable='customdata_publisher_node',
            name='customdata_publisher',
            output='screen',
            arguments=[LaunchConfiguration('sequence')],
            parameters=[{
                'publish_rate': 10.0,
                'dataset_root': '/home/ubuntu/custom_vo/mckimway'
            }]
        )
    ])
