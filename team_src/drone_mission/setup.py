from setuptools import setup

setup(
    name='drone_mission',
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/drone_mission']),
        ('share/drone_mission', ['package.xml']),
        ('share/drone_mission/launch', ['launch/square.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)
