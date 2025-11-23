import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'xai_navigation_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='noob',
    maintainer_email='howdoiusekeyboard@gmail.com',
    description='Explainable AI navigation with decision logging and explanations for HRI',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'xai_navigator_node = xai_navigation_pkg.xai_navigator_node:main',
        ],
    },
)
