import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'herbert2_robot'

setup(
    name=package_name,
    description='TODO: Package description',
    license='TODO: License declaration',
    version='0.0.1',

    maintainer='Tim Kingsmill',
    maintainer_email='tim.kingsmill@gmail.com',

    packages = find_packages(exclude=[]), 

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'), glob('param/*_params.yaml')),

        # Include all launch files. This is the most important line here!
        #(os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        #(os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,

    entry_points={
        'console_scripts': [
            'herbert2_robot = herbert2_robot.main:main'
        ],
    },
)
