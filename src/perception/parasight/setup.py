from setuptools import find_packages, setup

package_name = 'parasight'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/femur_shell.ply']),
        ('share/' + package_name + '/resource', ['resource/tibia_shell.ply']),
        ('share/' + package_name + '/resource', ['resource/plan_config.yaml']),
        ('share/' + package_name + '/resource', ['resource/plan_config_v2.yaml']),
        ('share/' + package_name + '/checkpoints', ['checkpoints/scaled_online.pth']),
        ('share/' + package_name + '/launch', ['launch/rs_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sreeharshaparuchur1',
    maintainer_email='venkata.surya@alumni.iiit.ac.in',
    description='Parasight ROS2 Package that integrates registration and tracking for assisted knee surgery',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'host = parasight.host:main',
            'cli = parasight.cli_client:main',
            'tracker = parasight.tracker:main',
            'snapshot = parasight.scripts.snapshot:main',
            'offline_register = parasight.scripts.offline_register:main'
        ],
    },
)
