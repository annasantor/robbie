from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santor689',
    maintainer_email='aisantor@ece.auth.gr',
    description='PID controller and CBF safety filter for 6-DOF manipulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Stage 1 — topic based CBF node
            'cbf_node = my_robot_controller.cbf_node:main',
        ],
    },
)