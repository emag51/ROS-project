from setuptools import find_packages, setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/my_robot_description']),
        ('share/my_robot_description', ['package.xml']),
        ('share/my_robot_description/launch', ['launch/robot_description.launch.py']),
        ('share/my_robot_description/urdf', ['urdf/my_robot.urdf']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emag51',
    maintainer_email='gamecd51@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
