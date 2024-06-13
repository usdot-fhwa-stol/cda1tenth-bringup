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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from datetime import datetime
import pathlib
import yaml
import os
import shutil

# This function is used to generate a command to record a ROS 2 rosbag that excludes topics
# topics as provided in the appropriate configuration file.

bag_dir = ''

def record_ros2_rosbag(context: LaunchContext, param_file):
    global bag_dir
    # Open vehicle config params file to process various rosbag settings
    with open(param_file, 'r') as f:

        params = yaml.safe_load(f)
        bag_dir = '/home/ubuntu/bags/rosbag2_' + str(datetime.now().strftime('%Y-%m-%d_%H%M%S'))
        proc = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_dir, *params['ros2_rosbag']['ros__parameters']['topics']],
            output='screen',
            shell='true'
            )

        return [proc]


def on_shutdown(event, context):
    params_file = os.path.join(get_package_share_directory("c1t_bringup"), "params", "params.yaml")
    shutil.copy(params_file, bag_dir)

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=record_ros2_rosbag, args=[os.path.join(get_package_share_directory("c1t_bringup"), "params", "params.yaml")]),
        RegisterEventHandler(
            OnShutdown(on_shutdown=on_shutdown)
        )
    ])
