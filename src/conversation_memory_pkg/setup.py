from setuptools import find_packages, setup

package_name = 'conversation_memory_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noob',
    maintainer_email='howdoiusekeyboard@gmail.com',
    description='Conversation memory and context management for HRI',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'conversation_memory_node = conversation_memory_pkg.conversation_memory_node:main',
            'explanation_handler = conversation_memory_pkg.explanation_handler:main',
        ],
    },
)
