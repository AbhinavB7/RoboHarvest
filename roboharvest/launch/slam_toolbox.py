from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': True},  # Set to True if using Gazebo
                {'publish_period_sec': 1.0},  # Adjust as needed
                {'odom_frame_id': 'odom'},
                {'base_frame_id': 'dummy_link'}, 
                {'base_link_frame_id': 'dummy_link'},
                # Add other slam_toolbox parameters here
            ],
        ),
    ])
