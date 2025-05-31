import os
from setuptools import setup

package_name = 'rplidar_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/rplidar_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='THITIPAT PREEDEDILOK',
    maintainer_email='gamecd51@gmail.com',
    description='Launch file for RPLIDAR',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
