import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


from launch_ros.actions import Node

def generate_launch_description():

  robot_type = 'create_2'

  pkg_create_bringup = get_package_share_directory('create_bringup')
  pkg_create_description = get_package_share_directory('create_description')
  pkg_teleop_twist_joy = get_package_share_directory('teleop_twist_joy')

  # Config
  joy_config = os.path.join(pkg_create_bringup, 'config/joystick',
                          'xbone.config.yaml')
  create_config = os.path.join(pkg_create_bringup, 'config/default.yaml')
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  use_rviz = LaunchConfiguration('use_rviz', default='true')

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
        '/include/state_publishers.launch.py'
    ]),
    launch_arguments={
        'use_sim_time': use_sim_time,
        'robot_type': TextSubstitution(text=str(robot_type))
    }.items(),
  )    

  rviz = Node(package='rviz2',
    executable='rviz2',
    arguments=['-d', os.path.join(pkg_create_description, 'rviz',
                                'create.rviz')],
    parameters=[{'use_sim_time': use_sim_time}],
    condition=IfCondition(use_rviz)
  )

  create2 = Node(
    package='create_driver',
    executable='create_driver',
    name='create_driver',
    output='screen',
    parameters=[{
          'config': create_config,
          'robot_model': 'CREATE_2',
          'use_sim_time': use_sim_time
  }])

  return LaunchDescription([
    DeclareLaunchArgument('use_sim_time',
                          default_value='false',
                          description='Use simulation clock if true'),
    DeclareLaunchArgument('use_rviz',
                          default_value='true',
                          description='Use rviz if true'),
    # Nodes
    joy_with_teleop_twist,
    state_publishers,
    rviz,
    create2
])