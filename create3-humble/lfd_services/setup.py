import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lfd_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), # include launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')) # include config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keonn',
    maintainer_email='pdas@keonn.com',
    description='Controls the lfd services ',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lfd_services = lfd_services.lfd_services:main' #name of the executable
        ],
    },
)