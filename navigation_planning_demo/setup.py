from glob import glob
from setuptools import setup

package_name = 'bolt_screw_nav2_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clara Jansson',
    maintainer_email='cjansson@tudelft.nl',
    description='Standalone Nav2 navigation/planning demo for the MIRTE bolts and screws sorting project.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sort_mission_planner = bolt_screw_nav2_demo.sort_mission_planner:main',
            'mock_mirte_base = bolt_screw_nav2_demo.mock_mirte_base:main',
        ],
    },
)