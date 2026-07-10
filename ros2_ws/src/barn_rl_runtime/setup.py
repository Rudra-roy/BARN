"""ament_python setup for barn_rl_runtime."""

from setuptools import find_packages, setup

package_name = 'barn_rl_runtime'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/rl_runtime.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barn-2027-prep contributors',
    maintainer_email='muztahid.appbaksho@gmail.com',
    description='CPU policy-inference runtime for the end-to-end RL track (Track B).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_runtime_node = barn_rl_runtime.rl_runtime_node:main',
        ],
    },
)
