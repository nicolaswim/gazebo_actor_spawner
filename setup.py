from setuptools import setup, find_packages
from glob import glob

package_name = 'gazebo_actor_spawner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
    (f'lib/{package_name}', ['scripts/spawn_actors']),  # ← wrapper for ros2 run
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Gazebo actor spawner',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'spawn_actors = gazebo_actor_spawner.gazebo_actor_relay:main',
        ],
    },
)
