from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    tx_node = Node(
        namespace= '_ns',
        package= 'workpiece_pkg',
        executable= 'simple_transmitter_node',     # name node in pkg writen in setup.py
        name= 'tx',                                # new node name
        remappings= [                              # change topic name
            ('/msg', '/topic_msg'),
            # ('/topic_name_1','/new_topic_name_1'),
            # ('/topic_name_2','/new_topic_name_2'),
        ]
    )

    rx_node = Node(
        namespace= '_ns',
        package= 'workpiece_pkg',
        executable= 'simple_receiver_node',
        name= 'rx',
        remappings= [
            ('/msg', '/topic_msg'),
            # ('/topic_name_1','/new_topic_name_1'),
            # ('/topic_name_2','/new_topic_name_2'),
        ]

    )

    ld.add_action(tx_node)
    ld.add_action(rx_node)
    return ld