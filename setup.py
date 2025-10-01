from glob import glob
from setuptools import find_packages, setup

package_name = 'gnss_waypoints_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yaw_float_to_imu = gnss_waypoints_nav.yaw_float_to_imu_node:main',
            'f9r_state_to_imu = gnss_waypoints_nav.f9r_state_to_imu_node:main',
            'gps_static_tf_pub = gnss_waypoints_nav.gps_static_tf_broadcaster:main',
        ],
    },
)
