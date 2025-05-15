from setuptools import setup

package_name = 'trash_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdamr',
    maintainer_email='you@example.com',
    description='YOLO + ToF 기반 쓰레기 감지 카메라 노드',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trash_camera_node = trash_camera.trash_camera_node:main',
        ],
    },
)
