import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'clean_table'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Modularized pick-and-place table-clearing behavior for Tiago',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator = clean_table.orchestrator_node:main',
            'drive_distance_server = clean_table.drive_distance_server:main',
            'align_to_marker_server = clean_table.align_to_marker_server:main',
            'marker_lookup_service = clean_table.marker_lookup_service:main',
            'pick_place_server = clean_table.pick_place_server:main',
        ],
    },
)
