from setuptools import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'slodda_bringup'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sigurdkroye-afk',
    maintainer_email='sigurd.kr.oye@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_robot = slodda_bringup.hello_robot:main',
            'astar_planner = slodda_bringup.astar_planner:main',
            'apf_controller = slodda_bringup.apf_controller:main',
            'obstacle_avoider = slodda_bringup.obstacle_avoider:main',
            'waypoint_mission = slodda_bringup.waypoint_mission:main',
        ],
    },
)
