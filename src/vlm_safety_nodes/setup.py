from setuptools import find_packages, setup

package_name = 'vlm_safety_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aryan',
    maintainer_email='aryan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zed_to_map_frame = vlm_safety_nodes.zed_to_map_transform:main',
            'vlm_preprocessor = vlm_safety_nodes.vlm_preprocessor:main',
            'vlm_query = vlm_safety_nodes.vlm_query:main',
        ],
    },
)
