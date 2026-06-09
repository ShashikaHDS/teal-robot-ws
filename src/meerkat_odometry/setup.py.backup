from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'meerkat_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meerkat',
    maintainer_email='meerkat@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'wheel_odometry = meerkat_odometry.wheel_odometry:main',
        'odom_transform = meerkat_odometry.odom_transform:main'
        ],
    },
)
