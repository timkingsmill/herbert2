from setuptools import find_packages
from setuptools import setup

package_name = 'herbert2_teleop'

setup(
    name=package_name,
    version='2.1.2',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='Tim Kingsmill',
    author_email='tim.kingsmill@gmail.com',
    maintainer='Tim Kingsmill',
    maintainer_email='tim.kingsmill@gmail.com',
    keywords=['ROS'],
    description=(
        'Teleoperation node using keyboard for Herbert2 robot.'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = herbert2_teleop.script.teleop_keyboard:main'
        ],
    },
)
