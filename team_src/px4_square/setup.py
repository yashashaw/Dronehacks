from glob import glob
from setuptools import setup

package_name = 'px4_square'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='noreply@example.com',
    description='Example square flown on PX4 position only (no VIO height, no depth).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'square_mission = px4_square.square_mission:main',
        ],
    },
)
