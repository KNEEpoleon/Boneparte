from setuptools import find_packages, setup

package_name = 'serialcomms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'pyserial'], #
    zip_safe=True,
    maintainer='kneepolean',
    maintainer_email='vrscar1@gmail.com',
    description='ROS 2 package for serial communication with Arduino',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'write_to_serial = serialcomms.write_to_serial:main',
            'testsub = serialcomms.testsub:main',
        ],
    },
)

