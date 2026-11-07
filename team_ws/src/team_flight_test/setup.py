from glob import glob
from setuptools import setup

package_name = 'team_flight_test'

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
    maintainer_email='team@example.com',
    description='Test-flight profiles (hover, yaw, out_back, box, course) with flight logging.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'flight_test = team_flight_test.flight_test_node:main',
        ],
    },
)
