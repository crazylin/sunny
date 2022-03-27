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
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with a component."""
    configFile = os.path.join(
        get_package_share_directory('camera_tis'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['camera_tis_node']['ros__parameters']

    camera_tis_node = ComposableNode(
        package='camera_tis',
        plugin='camera_tis::CameraTis',
        parameters=[configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile = os.path.join(
        get_package_share_directory('resize_image'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['resize_image_node']['ros__parameters']
        configParams['workers'] = 2

    resize_image_node = ComposableNode(
        package='resize_image',
        plugin='resize_image::ResizeImage',
        remappings=[('~/image', '/camera_tis_node/image')],
        parameters=[configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile = os.path.join(
        get_package_share_directory('rotate_image'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['rotate_image_node']['ros__parameters']
        configParams['workers'] = 2

    rotate_image_node = ComposableNode(
        package='rotate_image',
        plugin='rotate_image::RotateImage',
        remappings=[('~/image', '/resize_image_node/image_resized')],
        parameters=[configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile = os.path.join(
        get_package_share_directory('laser_line_center'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['laser_line_center_node']['ros__parameters']
        configParams['workers'] = 4

    laser_line_center_node = ComposableNode(
        package='laser_line_center',
        plugin='laser_line_center::LaserLineCenter',
        remappings=[('~/image', '/rotate_image_node/image_rotated')],
        parameters=[configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile = os.path.join(
        get_package_share_directory('line_center_reconstruction'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['line_center_reconstruction_node']['ros__parameters']
        configParams['workers'] = 2

    line_center_reconstruction_node = ComposableNode(
        package='line_center_reconstruction',
        plugin='line_center_reconstruction::LineCenterReconstruction',
        remappings=[('~/line', '/laser_line_center_node/line')],
        parameters=[configParams],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name='pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_tis_node,
            resize_image_node,
            rotate_image_node,
            laser_line_center_node,
            line_center_reconstruction_node],
        output='screen')

    configFile = os.path.join(
        get_package_share_directory('seam_tracking'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['seam_tracking_node']['ros__parameters']

    seam_tracking_node = Node(
        package='seam_tracking',
        executable='seam_tracking_node',
        remappings=[('~/pnts', '/line_center_reconstruction_node/pnts')],
        parameters=[configParams])

    configFile = os.path.join(
        get_package_share_directory('modbus'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['modbus_node']['ros__parameters']

    modbus_node = Node(
        package='modbus',
        executable='modbus_node',
        remappings=[('~/seam', '/seam_tracking_node/seam')],
        parameters=[configParams])

    configFile = os.path.join(
        get_package_share_directory('gpio_raspberry'),
        'config',
        'params.yaml')

    with open(configFile, 'r') as file:
        handle = yaml.safe_load(file)
        configParams = handle['gpio_raspberry_node']['ros__parameters']

    gpio_raspberry_node = Node(
        package='gpio_raspberry',
        executable='gpio_raspberry_node',
        parameters=[configParams])

    return launch.LaunchDescription([container,
        seam_tracking_node,
        modbus_node,
        gpio_raspberry_node])
