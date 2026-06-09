import os
from glob import glob
from setuptools import setup

package_name = 'meerkat_autostart'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'systemd'),
         glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teal',
    maintainer_email='teal@example.com',
    description='Autostart, GUI mode controller, and DS4 shortcuts.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mode_controller = meerkat_autostart.mode_controller:main',
        ],
    },
)
