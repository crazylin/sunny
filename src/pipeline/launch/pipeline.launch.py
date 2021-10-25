# Copyright 2019 Zhushi Tech, Inc.
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

import os
import yaml
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    """Generate launch description with a component."""
    configFile1 = os.path.join(
        get_package_share_directory('camera_mvs'),
        'config',
        'params.yaml')

    with open(configFile1, 'r') as file:
        configParams1 = yaml.safe_load(file)['camera_mvs_node']['ros__parameters']

    node1 = ComposableNode(
        package='camera_mvs',
        plugin='camera_mvs::CameraMvs',
        parameters=[configParams1],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile2 = os.path.join(
        get_package_share_directory('rotate_image'),
        'config',
        'params.yaml')

    with open(configFile2, 'r') as file:
        handle = yaml.safe_load(file)
        configParams2 = handle['rotate_image_node']['ros__parameters']

    node2 = ComposableNode(
        package='rotate_image',
        plugin='rotate_image::RotateImage',
        remappings=[('~/image', '/camera_mvs_node/image')],
        parameters=[configParams2],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name='pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[node1, node2],
        output='screen')

    return launch.LaunchDescription([container])
