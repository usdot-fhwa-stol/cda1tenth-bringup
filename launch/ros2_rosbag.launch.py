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
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown
from datetime import datetime
import os
import shutil

# This function is used to generate a command to record a ROS 2 rosbag that excludes topics
# topics as provided in the appropriate configuration file.

bag_dir = ''

def record_ros2_rosbag(context: LaunchContext):
    global bag_dir
    bag_dir = '/home/' + os.getlogin() + '/c1t_bags/rosbag2_' + str(datetime.now().strftime('%Y-%m-%d_%H%M%S'))
    proc = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir, '-a'],
        output='screen',
        )
    return [proc]


def on_shutdown(event, context):
    vehicle_name = LaunchConfiguration('vehicle').perform(context)
    params_file = os.path.join(get_package_share_directory("c1t_bringup"), "params", vehicle_name + "_params.yaml")
    shutil.copy(params_file, bag_dir)

def generate_launch_description():
    vehicle_launch_arg = DeclareLaunchArgument('vehicle')

    return LaunchDescription([
        OpaqueFunction(function=record_ros2_rosbag),
        RegisterEventHandler(
            OnShutdown(on_shutdown=on_shutdown)
        )
    ])
