from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='estimation_pkg',
            executable='imu_corrections',
            name='imu_correction_node',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.09831', '0', '0', '-0', '0', '0', 'base_link', 'imu_link']            
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_world',
            arguments=['0', '0', '0', '0', '0', '0', 'base', 'base_link']            
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',            
            arguments=['0', '0', '0', '-1.5708', '0', '1.5708', 'imu_link', 'imu']
        ),
        Node(
            package='imu_transformer',  # Replace with the actual package name
            executable='imu_transformer_node',
            name='imu_transformer_node',
            output='screen',
            parameters=[{'target_frame': 'imu_link'}],
            remappings=[
                ('/imu_in', '/data'),
                ('/imu_out', '/imu_rotated')
            ]
        ),
    ])
