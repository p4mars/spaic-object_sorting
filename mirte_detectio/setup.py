from setuptools import setup
import os
from glob import glob

package_name = 'mirte_detectio'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ruben',
    maintainer_email='ruben@example.com',
    description='Detection, motor control and arm nodes for MIRTE robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'Detection_Linux_v2      = mirte_detectio.Detection_Linux_v2_seperate:main',
            'Detection_motor_control = mirte_detectio.Detection_motor_control:main',
            'Detection_pickup_v2     = mirte_detectio.Detection_pickup_v2_seperate:main',
            'Detection_dropoff_motor = mirte_detectio.Detection_dropoff_motor:main',
            'Detection_dropoff       = mirte_detectio.Detection_dropoff_seperate:main',
        ],
    },
)