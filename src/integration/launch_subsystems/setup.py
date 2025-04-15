from setuptools import find_packages, setup

package_name = 'launch_subsystems'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_planning_hardware.py']),
        ('share/' + package_name + '/launch', ['launch/launch_planning_perception.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'pyserial'], #
    zip_safe=True,
    maintainer='kneepolean',
    maintainer_email='vrscar1@gmail.com',
    description='ROS 2 package to launch subsystems for boneparte',
    license='Apache-2.0',
    tests_require=['pytest'],
    # entry_points={
    #     'console_scripts': [
    #         'launch_planning_hardware = serialcomms.write_to_serial:main',
    #         'testsub = serialcomms.testsub:main',
    #     ],
    # },
)

