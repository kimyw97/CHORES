from setuptools import setup, find_packages

package_name = 'jdamr200_cartographer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name + '/launch', [
            'launch/cartographer.launch.py',
            'launch/occupancy_grid.launch.py',
            'launch/astar_planner.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/jdamr200_lidar.lua']),
        ('share/' + package_name + '/maps', ['maps/map.pgm', 'maps/map.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PDJ-sudo',
    maintainer_email='park619412@naver.com',
    description='SLAM + A* Path Planner with ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'astar_path_planner = jdamr200_cartographer.planner.astar_path_planner:main',
        ],
    },
)
