from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    my_ns = 'launch_ns'

    tx_node = Node(
        package= 'workpiece_pkg',
        namespace= my_ns,
        executable= 'simple_transmitter_node',     # name node in pkg writen in setup.py
        name= 'tx',                                # new node name
        output= 'both',                            #'screen'   "screen", "log", "both"
        # parameters=[{                           # not example
        #     "param_1":"text",
        #     "param_2": 55.4
        # }]
        remappings= [                     # change topic name
            ('/' + my_ns + '/msg', '/' + my_ns + '/topic_msg'),
            # ('/topic_name_1','/new_topic_name_1'),
            # ('/topic_name_2','/new_topic_name_2'),
        ]
    )

    rx_node = Node(
        package='workpiece_pkg',
        namespace= my_ns,
        executable='simple_receiver_node',
        name='rx',
        remappings=[                     # change topic name
            ('/' + my_ns + '/msg', '/' + my_ns + '/topic_msg'),
            # ('/topic_name_1','/new_topic_name_1'),
            # ('/topic_name_2','/new_topic_name_2'),
        ]

    )

    ld.add_action(tx_node)
    ld.add_action(rx_node)
    return ld