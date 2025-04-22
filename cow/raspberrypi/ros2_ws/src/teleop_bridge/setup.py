from setuptools import find_packages, setup

package_name = 'teleop_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdamr',
    maintainer_email='jdamr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_wasd_publisher = teleop_bridge.teleop_wasd_pub:main',
            'teleop_to_serial = teleop_bridge.teleop_to_serial:main',
        ],
    },
)
