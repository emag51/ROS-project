from setuptools import find_packages, setup

package_name = 'slam_toolbox_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/slam_toolbox_launch']),
        ('share/slam_toolbox_launch', ['package.xml']),
        ('share/slam_toolbox_launch/launch', ['launch/slam_toolbox.py']),
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
