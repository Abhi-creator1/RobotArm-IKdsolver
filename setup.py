from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [f'resource/{package_name}']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'visualization_msgs', 'interactive_markers', 'tf2_ros'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Description of my_robot_arm package',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'iksolver = my_robot_arm.iksolver:main',
            'interactive_marker_publisher = my_robot_arm.interactive_marker_publisher:main', 
        ],
    },
)
