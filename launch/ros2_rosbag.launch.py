# Copyright (C) 2023 LEIDOS.
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

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from datetime import datetime
import pathlib
import yaml
import os

# This function is used to generate a command to record a ROS 2 rosbag that excludes topics
# topics as provided in the appropriate configuration file.
def record_ros2_rosbag(context: LaunchContext, param_file):

    # Open vehicle config params file to process various rosbag settings
    with open(param_file, 'r') as f:

        params = yaml.safe_load(f)

        proc = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', '/home/ubuntu/bags/rosbag2_' + str(datetime.now().strftime('%Y-%m-%d_%H%M%S')), *params['ros2_rosbag']['ros__parameters']['topics']],
            output='screen',
            shell='true'
            )

        return [proc]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=record_ros2_rosbag, args=[os.path.join(get_package_share_directory("c1t_bringup"), "params", "params.yaml")])
    ])
