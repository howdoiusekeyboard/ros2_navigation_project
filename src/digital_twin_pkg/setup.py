from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'digital_twin_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noob',
    maintainer_email='howdoiusekeyboard@gmail.com',
    description='Digital twin monitoring and anomaly detection for HRI',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'digital_twin_monitor_node = digital_twin_pkg.digital_twin_monitor_node:main',
            'command_synchronizer_node = digital_twin_pkg.command_synchronizer_node:main',
        ],
    },
)
