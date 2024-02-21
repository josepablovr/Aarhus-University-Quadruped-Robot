from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
 return LaunchDescription([
 Node(
 package='testing_interface',
 executable='button_reader_node',
 output='screen'),
 ])
