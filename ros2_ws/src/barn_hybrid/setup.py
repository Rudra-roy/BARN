"""ament_python setup for barn_hybrid."""

from setuptools import find_packages, setup

package_name = 'barn_hybrid'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hybrid.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barn-2027-prep contributors',
    maintainer_email='muztahid.appbaksho@gmail.com',
    description='Hybrid arbiter: classical command + gated RL residual (Track C).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hybrid_node = barn_hybrid.hybrid_node:main',
        ],
    },
)
