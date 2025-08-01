import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'cartographer_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include our package files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.lua'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bob-YsPan',
    maintainer_email='someone@mail.com',
    description='Launch the Cartographer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_saver = cartographer_launcher.cartographer_map_saver:main',
        ],
    },
)
