# MIT License
#
# Copyright (c) Avery Girven
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # ROS packages
    pkg_create_gazebo = get_package_share_directory('create_gazebo')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    # Nodes
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch',
                         'ign_gazebo.launch.py')),
        launch_arguments={
            'ign_args': '-r ' + pkg_create_gazebo + '/worlds/test.sdf'
        }.items(),
    )

    ign_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/test/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/create/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/create/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/create/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/model/create/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'
        ],
        output='screen',
        remappings=[
            ('/world/test/clock', '/clock'),
            ('/model/create/tf', '/tf'),
            ('/model/create/cmd_vel', '/robot/cmd_vel'),
            ('/model/create/odometry', '/odom'),
            ('/model/create/joint_state', 'joint_states'),
            ('/lidar', '/create/raw_scan'),
            ('/lidar/points', '/create/raw_points'),
            ('/imu', '/create/imu'),
        ])

    ign_spawn_robot = Node(package='ros_ign_gazebo',
                           executable='create',
                           arguments=[
                               '-name', 'create', '-x', '5', '-y', '3.20', '-Y',
                               '3.14', '-topic', 'robot_description'
                           ],
                           output='screen')

    return LaunchDescription([
        # Nodes
        ign_gazebo,
        ign_bridge,
        ign_spawn_robot,
    ])