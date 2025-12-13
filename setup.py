from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_actor_spawner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copy Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Copy World Files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Copy URDF Files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Copy RViz Configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Dynamic Gazebo Actor Spawner',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_actors = gazebo_actor_spawner.spawn_actors:main',
            'gazebo_actor_relay = gazebo_actor_spawner.gazebo_actor_relay:main',
        ],
    },
)