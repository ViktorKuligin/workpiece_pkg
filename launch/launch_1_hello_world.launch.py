from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='workpiece_pkg',
            namespace='launch_ns',
            executable='simple_hello_world_node', # node name in setup.py 
            name='helloWorld',                    # new node name
        )
    ])