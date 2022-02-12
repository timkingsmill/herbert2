import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration

import launch_ros.actions

# ---------------------------------------------------------------

def generate_launch_description():

    # -----------------------------------------------------------

    print('Herbert2 Cartographer Launch')

    # -----------------------------------------------------------

    ld = LaunchDescription()

    # --------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------

    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')

    # --------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------
    # Setup the occupancy grid launch file

    # Get the config directory
    herbert2_cartographer_prefix = get_package_share_directory('herbert2_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', 
                              default=os.path.join(herbert2_cartographer_prefix, 'config'))

    conf_path_argument = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value = cartographer_config_dir,
        description = 'Full path to config file to load'
    )
    ld.add_action(conf_path_argument)
    
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='herbert2_config.lua')
    conf_basename_argument = DeclareLaunchArgument(
        'configuration_basename',
        default_value = configuration_basename,
        description = 'Name of lua file for cartographer'
    )
    ld.add_action(conf_basename_argument)

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'false',
        description = 'Use simulation (Gazebo) clock if true'
    )
    ld.add_action(use_sim_time_argument)
    
    # Launch the cartographer ROS node
    ros_cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        emulate_tty=True,
        parameters = [{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]
    )
    ld.add_action(ros_cartographer_node)
    
    # --------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------
    # Setup the occupancy grid launch file
    resolution = LaunchConfiguration('resolution', default='0.05')

    resolution_argument = DeclareLaunchArgument(
        'resolution',
        default_value = resolution,
        description = 'Resolution of a grid cell in the published occupancy grid'
    )
    ld.add_action(resolution_argument)

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    publish_period_sec_argument = DeclareLaunchArgument(
        'publish_period_sec',
        default_value = publish_period_sec,
        description = 'OccupancyGrid publishing period'
    )
    ld.add_action(publish_period_sec_argument)

    # include the occupancy launch description file
    occupancy_launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
    )
    ld.add_action(occupancy_launch_description)

    # --------------------------------------------------------------------------------
    # --------------------------------------------------------------------------------
    # Setup and launch the RViz2 ROS node

    rviz_config_dir = os.path.join(get_package_share_directory('herbert2_cartographer'),
                                   'rviz', 'herbert2_cartographer.rviz')
    # Launch the rviz2 ROS node
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',      
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(rviz2_node)

    # -----------------------------------------------------------

    return ld

# ---------------------------------------------------------------
