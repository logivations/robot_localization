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
import json
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import configparser
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

SIMULATION_PARAMETERS_PATH = "/data/workspace/deep_cv/appconfig/tracking/simulation_parameters.ini"

def generate_launch_description():

    simulation_parameters = configparser.RawConfigParser()
    simulation_parameters.read(SIMULATION_PARAMETERS_PATH)
    agv_ids = json.loads(simulation_parameters.get("conf", "agv_ids"))

    params = yaml.load(open("/code/ros2_ws/src/robot_localization/params/ekf.yaml"))

    launch_description = LaunchDescription()
    for agv_id in agv_ids:
        file_path = "/code/ros2_ws/src/robot_localization/params/simulated_agv" + str(agv_id) + "_ekf.yaml"
        params["ekf_filter_node"]["ros__parameters"]["odom0"] = "/agv" + str(agv_id) + "/wheel_odom_node/odom_wheel"
        params["ekf_filter_node"]["ros__parameters"]["odom1"] = "/agv" + str(agv_id) + "/camera_odom_node/odom_camera"
        with open(file_path, 'w') as f:
            yaml.dump(params, f)
        launch_description.add_entity(
            launch_ros.actions.Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                remappings=[
                    ('/odometry/filtered', ('/agv' + str(agv_id) + '/odom')),
                    ('/accel/filtered', 'acceleration/filtered'),
                ],
                parameters=[file_path],
            )
        )

    return launch_description
