from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([
 Node(
 package='gait_pkg',
 executable='crawl_gait',
 output='screen'),
 ])
