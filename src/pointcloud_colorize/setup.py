from glob import glob
from setuptools import setup

package_name = 'pointcloud_colorize'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teal',
    maintainer_email='teal@example.com',
    description='Colorize a LiDAR cloud using a camera image.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'colorize_node = pointcloud_colorize.colorize_node:main',
        ],
    },
)
