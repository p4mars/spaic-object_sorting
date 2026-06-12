from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mirte_sorting'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AE4ASM527 Group 2',
    maintainer_email='student@tudelft.nl',
    description='Unified MIRTE sorting robot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Detection / perception
            'perception_node = mirte_sorting.perception_node:main',
            'pickup_node = mirte_sorting.pickup_node:main',
            'dropoff_node = mirte_sorting.dropoff_node:main',
            # Localisation
            'localisation_input_node = mirte_sorting.localisation_input_node:main',
            'localisation_output_node = mirte_sorting.localisation_output_node:main',
            'hybrid_localiser = mirte_sorting.hybrid_localiser:main',
            # Mapping helpers
            'camera_info_sync_node = mirte_sorting.camera_info_sync_node:main',
            'semantic_map_node = mirte_sorting.semantic_map_node:main',
            # Mission planner
            'mission_planner_node = mirte_sorting.mission_planner_node:main',
        ],
    },
)
