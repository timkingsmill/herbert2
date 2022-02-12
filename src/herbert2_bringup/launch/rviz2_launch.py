# --------------------------------------------------------------------
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

# --------------------------------------------------------------------

def generate_launch_description():

    ld = LaunchDescription()
    
    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
    
    rviz_config_dir = os.path.join(
        get_package_share_directory('herbert2_description'),
                                    'rviz',
                                    'herbert2_robot.rviz')

    # ----------------------------------------------------------------

    rviz2_node = launch_ros.actions.Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', rviz_config_dir],
    ) 
    ld.add_action(rviz2_node)
    
    # ----------------------------------------------------------------

    return ld

    # ----------------------------------------------------------------
    # ----------------------------------------------------------------
