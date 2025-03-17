from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    tx_node = Node(
        package= 'workpiece_pkg',
        executable= 'simple_param_node',     # node name in setup.py
        name= 'param_tx',                    # new node name
        parameters=[{
            "topic_name":"message",
            "timer_period": 0.2,
            "print_available": True,
            "fix_data": 45,
        }]
    )

    ld.add_action(tx_node)
    return ld