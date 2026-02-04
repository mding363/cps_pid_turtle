from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cps_pid_turtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share',package_name,'launch'), glob('launch/follow_shape_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'controller = cps_pid_turtle.controller:main',
		'sensor_publisher = cps_pid_turtle.sensor_publisher:main',
        ],
    },
)
