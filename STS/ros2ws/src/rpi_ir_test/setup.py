from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rpi_ir_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files (if you had any, none are here for simplicity)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 package for testing a digital IR sensor on Raspberry Pi GPIO.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Defines the executable name 'ir_sensor_node' that runs the
            # main function inside the 'ir_sensor_node.py' file.
            'ir_sensor_node = rpi_ir_test.ir_sensor_node:main'
        ],
    },
)
