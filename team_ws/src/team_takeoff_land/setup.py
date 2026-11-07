from glob import glob
from setuptools import setup

package_name = 'team_takeoff_land'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='LucasKniska',
    maintainer_email='lucaskniska1@gmail.com',
    description='Takeoff/land services that cut the planner off.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'takeoff_land = team_takeoff_land.takeoff_land_node:main',
            'takeoff_land_mission = '
            'team_takeoff_land.takeoff_land_mission:main',
        ],
    },
)
