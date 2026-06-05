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
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santor689',
    maintainer_email='aisantor@ece.auth.gr',
    description='PID controller and CBF safety filter for 6-DOF arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cbf_node = my_robot_controller.cbf_node:main',
            'collision_objects_node = my_robot_controller.collision_objects_node:main',
        ],
    },
)