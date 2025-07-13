from setuptools import find_packages, setup
import os

package_name = 'realsense'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),


        ('share/' + package_name + '/launch', [
            'launch/visual_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='work',
    maintainer_email='work@todo.todo',
    description='RealSense 影像處理與檢測功能包',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'box_edge_detect = realsense.box_edge_detect:main',
            'color_detect = realsense.color_detect:main',
            'realsense_read = realsense.realsense_read:main',
            'yolo_number_detect = realsense.yolo_number_detect:main',
            'color_range = realsense.color_range:main',
            'depth_calculate = realsense.depth_calculate:main',
            '2d_plane = realsense.2d_plane:main',
            'paddleorc = realsense.paddleorc:main',
        ],
    },
)
