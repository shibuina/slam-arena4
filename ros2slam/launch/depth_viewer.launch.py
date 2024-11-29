from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    depth_node = Node(
        package='ros2slam',
        executable='depth_publisher',
        name='depth_publisher',
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        depth_node,
        rviz_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=depth_node,
                on_exit=[rviz_node]
            )
        )
    ])
