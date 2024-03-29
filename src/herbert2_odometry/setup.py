from setuptools import setup

package_name = 'herbert2_odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='herbert',
    maintainer_email='tim.kingsmill@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_publisher = herbert2_odometry.odometry_publisher:main',
            'odometry_logger = herbert2_odometry.odometry_logger:main',
        ],
    },
)
