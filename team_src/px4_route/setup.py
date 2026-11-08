from glob import glob
from setuptools import setup

package_name = 'px4_route'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/routes', glob('routes/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='noreply@example.com',
    description='Waypoint route follower on PX4 position (px4_square stack + YAML route + carrot).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'route_mission = px4_route.route_mission:main',
        ],
    },
)
