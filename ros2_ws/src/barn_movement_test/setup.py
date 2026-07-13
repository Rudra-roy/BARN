"""ament_python setup for barn_movement_test."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'barn_movement_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barn-2027-prep contributors',
    maintainer_email='muztahid.appbaksho@gmail.com',
    description='Minimal fixed-duration movement and odometry test planner.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_and_odom_test = '
            'barn_movement_test.movement_and_odom_test_node:main',
        ],
    },
)
