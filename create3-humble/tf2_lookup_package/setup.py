import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'tf2_lookup_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share',package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools', 'os', 'glob'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='simaya13@gmail.com',
    description='Tf2 lookup',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['map_base_link_frame_listener = tf2_lookup_package.tf2_pose_2D_listener:main'
        ],
    },
)