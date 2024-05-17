from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([
 Node(
 package='estimation_pkg',
 executable='state_estimation',
 output='screen'),
 ])
