from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    tx_node = Node(
        package= 'workpiece_pkg',
        executable= 'joy_param_node',     # node name in setup.py
        name= 'joy_param',                # new node name
        parameters=[{
            "main_dictionary": "green",     # red or green
        }]
    )

    ld.add_action(tx_node)
    return ld