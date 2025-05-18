from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'trash_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # ✅ 핵심 포인트!
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Trash detection and distance publishing',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trash_camera_node = trash_camera.trash_camera_node:main'
        ],
    },
)
