from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localisation_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='reiaramkumar',
    maintainer_email='reia.ramkumar.nl@gmail.com',
    description='creating the localisation package for mirte to determine its current position',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation_input_node  = localisation_pkg.localisation_input_node:main',
            'localisation_output_node = localisation_pkg.localisation_output_node:main',
            'hybrid_localiser         = localisation_pkg.hybrid_localiser:main',
            'apriltag_corrector_at    = localisation_pkg.apriltag_corrector_at:main',
        ],
    },
)

