from glob import glob
from setuptools import setup

package_name = 'course_pipeline'

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
    maintainer='DroneHacks team',
    maintainer_email='team@example.com',
    description='Landing-pad detector, pad guide and placeholder planner.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pad_detector = course_pipeline.pad_detector:main',
            'pad_guide = course_pipeline.pad_guide:main',
            'path_placeholder = course_pipeline.path_placeholder:main',
        ],
    },
)
