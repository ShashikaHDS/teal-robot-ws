from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'meerkat_motors'

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
             'oriental_ros = meerkat_motors.oriental_ros:main',
             'dualshock_ros = meerkat_motors.dualshock_ros:main'
        ],
    },
)
