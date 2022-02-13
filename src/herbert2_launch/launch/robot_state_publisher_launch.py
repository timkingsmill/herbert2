import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions

import xacro

def generate_launch_description():

    ld = LaunchDescription()

    # ------------------------------------------------------
    # ------------------------------------------------------
    #  Get the Robot Definition File.
    urdf_file = os.path.join(
        get_package_share_directory('herbert2_description'),
                                    'urdf',
                                    'herbert2_robot.urdf')

    # Read the file.
    robot_doc = xacro.parse(open(urdf_file))
    xacro.process_doc(robot_doc)
    # Define the Robot description parameters.
    robot_description = {'robot_description': robot_doc.toxml()}

    #with open(urdf_file, 'r') as infp:
    #    robot_desc = infp.read()


    # Prepare the robot state publisher driver node.
    robot_state_publisher = launch_ros.actions.Node(
        name = 'robot_state_publisher',
        package = 'robot_state_publisher', 
        executable = 'robot_state_publisher', 
        output = 'screen',
        parameters = [robot_description]
    )
    ld.add_action(robot_state_publisher)

    # ------------------------------------------------------
    # ------------------------------------------------------

    return ld

# ----------------------------------------------------------
# ----------------------------------------------------------
