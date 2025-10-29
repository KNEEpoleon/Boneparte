from setuptools import find_packages, setup

package_name = 'tcp_server_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/avp_server_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='genisis',
    maintainer_email='dakshadhar1810@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tcp_server_node = tcp_server_pkg.tcp_server_node:main',
            'avp_tcp_server = tcp_server_pkg.avp_tcp_server:main',
        ],
    },
)
