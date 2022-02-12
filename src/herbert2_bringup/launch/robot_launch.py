import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration

import launch_ros.actions

# --------------------------------------------------------------------
# --------------------------------------------------------------------

def generate_launch_description():

    # ------------------------------------------------------

    ld = launch.LaunchDescription()

    """
    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
    # Publish the robots state.
    # Includes the urdf robot description file.
    robot_publisher_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robot_state_publisher_launch.py'])
    )
    ld.add_action(robot_publisher_launch_description)
    """

    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
    # Launch the RPLidar ROS2 node. 
    """
    ********     Run this node on the h2-mapping raspberry pi      ******************

    rplidar_launch_dir = LaunchConfiguration(
        # Find the RPLidars launch file directory.
        'rplidar_launch_dir',
        default = os.path.join(
            get_package_share_directory('herbert2_rplidar'), 'launch')
    )

    rplidar_launch_description = IncludeLaunchDescription(
        # Add the RPLidar node's launch description file.
        PythonLaunchDescriptionSource([rplidar_launch_dir, '/rplidar_launch.py'])
    )
    ld.add_action(rplidar_launch_description)
    """

    # ------------------------------------------------------
    # Prepare the Herbert2 ODrive Robot Node.

    herbert2_robot_param_dir = LaunchConfiguration(
        'herbert2_robot_param_dir',
        default = os.path.join(
            get_package_share_directory('herbert2_robot'), 'param', 'robot_params.yaml')
    )

    # ------------------------------------------------------

    herbert2_robot = launch_ros.actions.Node(
        name = 'herbert2_robot',
        package = 'herbert2_robot', 
        executable = 'herbert2_robot', 
        output = 'screen',
        emulate_tty=True,
        parameters = [herbert2_robot_param_dir]
    )
    ld.add_action(herbert2_robot)

    # ----------------------------------------------------------------
    # Launch the IMU Driver ROS2 node. 
    
    #********     Run this node on the h2-mapping raspberry pi      ******************
    # Prepare the odrive driver node.
    """
    imu_driver_node_description = launch_ros.actions.Node(
        name = 'herbert2_imu_driver_node',
        package = 'herbert2_imu', 
        executable = 'imu_driver', 
        output = 'screen',
        emulate_tty = True,
    )
    ld.add_action(imu_driver_node_description)
    """

    # ----------------------------------------------------------------
    # Launch the odometry ROS2 node. 
    """
    odometry_node_description = launch_ros.actions.Node(
        name = 'herbert2_odometry_node',
        package = 'herbert2_odometry', 
        executable = 'odometry', 
        output = 'screen',
        emulate_tty = True,
    )
    ld.add_action(odometry_node_description)
    """

    # ------------------------------------------------------

    
    return ld


# --------------------------------------------------------------------
# --------------------------------------------------------------------

