import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('motor_driver/launch/*.py')),  # <== NOTE the path!
            (os.path.join('share', package_name, 'urdf'), glob('motor_driver/urdf/*.urdf')),  # <== NOTE the path!

    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='abubakr',
    maintainer_email='abubakr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'motor_controller = motor_driver.motor_controller:main'
            #'serial_test = motor_driver.serial_test:main',
            'motor_serial_node = motor_driver.motor_serial_node:main',
            'twist_to_motors = motor_driver.twist_to_motors:main',
            'odom_node = motor_driver.odom_node:main',
        ],
    },
)
