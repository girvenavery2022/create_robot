import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


from launch_ros.actions import Node

def generate_launch_description():

  robot_type = 'roomba_400'

  pkg_create_bringup = get_package_share_directory('create_bringup')
  pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

  # Config
  joy_config = os.path.join(pkg_create_bringup, 'config/joystick',
                          'xbone.config.yaml')

  create_config = os.path.join(pkg_create_bringup, 'config/default.yaml')

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')


  joy_with_teleop_twist = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_teleop_twist_joy, 'launch', 'teleop-launch.py')),
    launch_arguments={
        'joy_config': 'xbox',
        'joy_dev': '/dev/input/js0',
        'config_filepath': joy_config
    }.items(),
  )

  state_publishers = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(pkg_create_bringup, 'launch'),
        '/include/state_publishers/state_publishers.launch.py'
    ]),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'robot_type': TextSubstitution(text=str(robot_type))
    }.items(),

  )    

  roomba_400 = Node(
    package='create_driver',
    executable='create_driver',
    name='create_driver',
    output='screen',
    parameters=[{
          'config': create_config,
          'robot_model': 'ROOMBA_400',
          'use_sim_time': use_sim_time
  }])

  return LaunchDescription([
    DeclareLaunchArgument('use_sim_time',
                          default_value='false',
                          description='Use simulation clock if true'),
    # Nodes
    joy_with_teleop_twist,
    state_publishers,
    roomba_400
])