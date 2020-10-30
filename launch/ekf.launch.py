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
from nav2_common.launch import RewrittenYaml


from configparser import ConfigParser

config = ConfigParser()
config.read("/data/workspace/deep_cv/appconfig/tracking/agv_id.ini")
AGV_ID = config.getint("conf", "agv_id")

def generate_launch_description():
    namespace = str ("agv" + str(AGV_ID))
    configured_params = RewrittenYaml(
        source_file='/code/ros2_ws/src/robot_localization/params/ekf.yaml',
        root_key=namespace,
        param_rewrites={},
        convert_types=True)
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('/odometry/filtered', ("/" + namespace +'/odom')),
                ('/accel/filtered', 'acceleration/filtered'),
            ],
            parameters=[configured_params,
                        {'odom0': '/'+namespace+'/wheel_odom_node/odom_wheel'},
                        {'odom1': '/'+namespace+'/camera_odom_node/odom_camera'}],
        ),
    ])
