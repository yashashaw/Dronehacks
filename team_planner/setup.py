from glob import glob
from setuptools import find_packages, setup

package_name = 'team_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='DroneHacks team',
    maintainer_email='team@example.com',
    description='Depth-based path planner publishing /course/goal.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['planner_node = team_planner.planner_node:main']},
)
