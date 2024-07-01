import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'lfd_pose_package'

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
    description='Lfd pose generator',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['map_base_link_frame_listener = lfd_pose_package.tf2_pose_2D_listener:main', 
                            'lfd_pose_generator=lfd_pose_package.lfd_pose:main',
                            'nav2_api=lfd_pose_package.nav2_api:main'
        ],
    },
)