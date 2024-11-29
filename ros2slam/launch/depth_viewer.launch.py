import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        Node(
            package='ros2slam',
            executable='depth_publisher',
            name='depth_publisher_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('/depth_image', '/camera/depth/image_raw')]
        ),
    ])
