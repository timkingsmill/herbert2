"""
    This launch file executes all ROS nodes on the 
    Raspberry Pi that drives IMU.
"""

from launch import LaunchDescription
from launch_ros.actions import Node as NodeDescription

#from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()

    # ----------------------------------------------------------------
    # Launch the IMU Driver ROS2 node. 
    imu_driver_node_description = NodeDescription(
        name = 'herbert2_imu_driver_node',
        package = 'herbert2_imu', 
        executable = 'imu_driver', 
        output = 'screen',
        emulate_tty = True,
    )
    ld.add_action(imu_driver_node_description)

    # ----------------------------------------------------------------
    # Launch the Odometery Publisher ROS2 node. 
    odometry_publisher_description = NodeDescription(
        name = 'herbert2_odometry_publisher',
        package = 'herbert2_odometry', 
        executable = 'odometry_publisher', 
        output = 'screen',
        emulate_tty = True,
        #arguments=['--ros-args', '--log-level', 'debug']
    )
    ld.add_action(odometry_publisher_description)

    # ----------------------------------------------------------------
    # Launch the Odometery Logger ROS2 node. 
    odometry_logger_description = NodeDescription(
        name = 'herbert2_odometry_logger',
        package = 'herbert2_odometry', 
        executable = 'odometry_logger', 
        output = 'screen',
        emulate_tty = True,
        #arguments=['--ros-args', '--log-level', 'debug']
    )
    ld.add_action(odometry_logger_description)

    # ----------------------------------------------------------------




    return ld

    # ----------------------------------------------------------------
    # ----------------------------------------------------------------

