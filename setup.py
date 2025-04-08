import os                                                # by launch
from glob import glob                                    # by launch

from setuptools import find_packages, setup

package_name = 'workpiece_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')), # by launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vi',
    maintainer_email='kuligin.viktor.alexandrovich@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            "joy_red_node = workpiece_pkg.joy_1_red:main",
            "joy_green_node = workpiece_pkg.joy_2_green:main",
            "joy_param_node = workpiece_pkg.joy_3_param:main",
            "joy_simulator_node = workpiece_pkg.joy_simulator:main",

            "pg_keyboard = workpiece_pkg.pg_1_keyboard:main",
            "pg_simulator = workpiece_pkg.pg_2_joy_simulator:main",
            "pg_direction = workpiece_pkg.pg_3_move_number_direction:main",

            "cam_reader_node = workpiece_pkg.cam_0_reader:main",
            "cam_create_node = workpiece_pkg.cam_0_create_black_rect:main",
            "cam_rgb_node = workpiece_pkg.cam_0_create_rgb:main",
            "cam_paint_node = workpiece_pkg.cam_1_paint:main",
            "cam_hsv_node = workpiece_pkg.cam_2_hsv:main",
            "cam_hsv2_node = workpiece_pkg.cam_2_hsv_2:main",
            "cam_search_contour_hsv_node = workpiece_pkg.cam_3_search_contour_by_hsv:main",
            "cam_aruco = workpiece_pkg.cam_4_aruco:main",
            "cam_aruco_create = workpiece_pkg.cam_4_aruco_create:main",
            "cam_aruco_detect = workpiece_pkg.cam_4_aruco_detect:main",

            "simple_hello_world_node = workpiece_pkg.simple_1_hello_world:main",
            "simple_transmitter_node = workpiece_pkg.simple_2_transmitter:main",
            "simple_receiver_node = workpiece_pkg.simple_2_receiver:main",
            "simple_param_node = workpiece_pkg.simple_3_param:main",

            "rviz_point_node = workpiece_pkg.rviz_1_point:main",
            "rviz_point_move_node = workpiece_pkg.rviz_1_point_move:main",
            "rviz_range_node = workpiece_pkg.rviz_2_range:main",
            "rviz_pose_node = workpiece_pkg.rviz_3_pose:main",
            "rviz_marker_single_node = workpiece_pkg.rviz_4_marker_single:main",
        ],
    },
)
