import os
from   ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    ld = launch.LaunchDescription()


    remaptopics = [('/robot/rplidar_scan', '/scan')]

    rplidar_publisher_config = os.path.join(
        get_package_share_directory('herbert2_rplidar'),
        'config',
        'rplidar_params.yaml'
    )

    # Prepare the odrive driver node.
    rplidar_publisher = launch_ros.actions.Node(
        name = 'rplidar_publisher',
        package = 'herbert2_rplidar', 
        executable = 'rplidar_driver', 
        output = 'screen',
        emulate_tty = True,
        parameters = [rplidar_publisher_config],
        remappings=remaptopics,
    )

    ld.add_action(rplidar_publisher)

    return ld
    