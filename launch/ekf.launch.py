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
    odom0 = "odom0: /agv" + str(AGV_ID) +  "/wheel_odom_node/odom"
    odom1 =  "odom1: /agv"+ str(AGV_ID) + "/camera_odom_node/odom"
    cmd_start = (
            "sed -i -E 's;odom0:.+;"
            + odom0
            + ";g' "
            + PARAMETER_FILE_PATH
    )
    cmd_end = (
            "sed -i -E 's;odom1:.+;" + odom1 + ";g' " + PARAMETER_FILE_PATH
    )
    try:
        subprocess.check_output(
            cmd_start, timeout=2, shell=True,
        )
        subprocess.check_output(
            cmd_end, timeout=2, shell=True,
        )
    except:
        logger.error(f"could not modify ekf params file. ")
    
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'
        ),
        
        #PushRosNamespace(namespace=namespace),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('/odometry/filtered', ('/agv' + str(AGV_ID) + '/odom')),
                ('/accel/filtered', 'acceleration/filtered'),
            ],
            parameters=["/code/ros2_ws/src/robot_localization/params/ekf.yaml"],
        ),
    ])
