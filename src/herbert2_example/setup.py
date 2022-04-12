import os
from glob import glob
from setuptools import setup

# ------------------------------------------------------------------------------------

package_name = 'herbert2_example'

# ------------------------------------------------------------------------------------

setup(
    name=package_name,
    version='0.0.0',
    maintainer='herbert',
    maintainer_email='tim.kingsmill@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    zip_safe=True,
    install_requires=['setuptools'],

    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    ],

    entry_points={
        'console_scripts': [
            'obstacle_detection_node = \
                herbert2_example.herbert2_obstacle_detection.main:main',
            'herbert2_position_control = \
                herbert2_example.herbert2_position_control.main:main',
            'herbert2_rotation_control = \
                herbert2_example.herbert2_rotation_control.main:main',
            'herbert2_trajectory_control = \
                herbert2_example.herbert2_trajectory_control.main:main',
        ],
    },
)

# ------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------
