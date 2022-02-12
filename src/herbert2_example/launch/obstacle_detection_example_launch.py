import os
from   ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

# -----------------------------------------------------------------------

def generate_launch_description():

    ld = LaunchDescription()

    # -------------------------------------------------------------------
    # Get the example packages config files
    herbert_examples_config = os.path.join(
        get_package_share_directory('herbert2_example'),
        'config',
        'params.yaml'
    )

    # -------------------------------------------------------------------
    # Prepare the obstacle detection node.
    obstacle_detection_node = launch_ros.actions.Node(
        name = 'obstacle_detector',
        package = 'herbert2_example', 
        executable = 'obstacle_detection_node', 
        output = 'screen',
        emulate_tty = True,
        parameters = [herbert_examples_config]
    )
    ld.add_action(obstacle_detection_node)

    # -------------------------------------------------------------------
    # Start the Herbert2 Robot node.
    robot_launch_path = os.path.join(
        get_package_share_directory('herbert2_bringup'), 'launch', 'robot_launch.py'
    )

    robot_launch_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_launch_path]),
        launch_arguments= {'node_name': 'bar'}.items()
    )

    ld.add_action(robot_launch_description)

    # -------------------------------------------------------------------

    return ld
    
# -----------------------------------------------------------------------
