
import os
import launch 

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration

import launch_ros.actions


# --------------------------------------------------------------------

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # ----------------------------------------------------------------

    ld = LaunchDescription()

    # ----------------------------------------------------------------
    
    resolution_argument = DeclareLaunchArgument(
        'resolution',
        default_value = resolution,
        description = 'Resolution of a grid cell in the published occupancy grid'
    )
    ld.add_action(resolution_argument)

    publish_period_sec_argument = DeclareLaunchArgument(
        'publish_period_sec',
        default_value = publish_period_sec,
        description = 'OccupancyGrid publishing period'
    )
    ld.add_action(publish_period_sec_argument)

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'false',
        description = 'Use simulation (Gazebo) clock if true'
    )
    ld.add_action(use_sim_time_argument)

    # ----------------------------------------------------------------

    occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
    )
    ld.add_action(occupancy_grid_node)

    # ----------------------------------------------------------------
  
    return ld

# --------------------------------------------------------------------
# --------------------------------------------------------------------

