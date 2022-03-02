"""
    This launch file executes all ROS nodes on the
    Raspberry Pi that drives the ODrive controllers and motors.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node as NodeDescription
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------

def generate_launch_description():

    ld = LaunchDescription()

    # ------------------------------------------------------
    # Prepare the Herbert2 ODrive Robot Node.
    herbert2_robot_param_dir = LaunchConfiguration(
        'herbert2_robot_param_dir',
        default = os.path.join(
            get_package_share_directory('herbert2_robot'), 'param', 'robot_params.yaml')
    )

    # ------------------------------------------------------

    herbert2_robot = NodeDescription(
        name = 'herbert2_robot',
        package = 'herbert2_robot', 
        executable = 'herbert2_robot', 
        output = 'screen',
        emulate_tty=True,
        parameters = [herbert2_robot_param_dir]
    )

    ld.add_action(herbert2_robot)

    # ----------------------------------------------------------------

    return ld

    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
