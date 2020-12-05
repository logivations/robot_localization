# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
import subprocess
from logging import getLogger
logger = getLogger(__name__)

from configparser import ConfigParser

config = ConfigParser()
config.read("/data/workspace/deep_cv/appconfig/tracking/agv_id.ini")
AGV_ID = config.getint("conf", "agv_id")

PARAMETER_FILE_PATH = "/code/ros2_ws/src/robot_localization/params/ekf.yaml"

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'rviz_prefix', default_value='',
            description='Rviz prefix'
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('/odometry/filtered', (LaunchConfiguration("namespace") , '/odom')),
                ('/odom0', (LaunchConfiguration("namespace") ,  '/', LaunchConfiguration("rviz_prefix"),'wheel_odom_node/odom')),
                ('/odom1',  (LaunchConfiguration("namespace") ,'/', LaunchConfiguration("rviz_prefix"), 'camera_odom_node/odom')),
                ('/costmap_node/map',  ("/", LaunchConfiguration("rviz_prefix") , 'costmap_node/map')),
            ],
            parameters=["/code/ros2_ws/src/robot_localization/params/ekf.yaml"],
        ),
    ])
