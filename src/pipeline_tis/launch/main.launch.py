"""
Generate a launch description.

This launch configuration gathers a bunch of node into a pipeline:
It parses the underlay config: .params.yaml and overlay config: params.yaml.
The overlay is updated to the underlay.
Several computation intensive nodes employee more workers.
Image related topics are passed around via intra-process communication.
Only pointers to image are copied to minimize CPU consumption.
"""

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
    config = os.path.join(
        get_package_share_directory('config_tis'),
        'config',
        '.params.yaml')
    with open(config, 'r') as file:
        params = yaml.safe_load(file)

    try:
        configNew = os.path.join(
            get_package_share_directory('config_tis'),
            'config',
            'params.yaml')
        with open(configNew, 'r') as file:
            paramsNew = yaml.safe_load(file)
        params.update(paramsNew)
    except Exception:
        pass

    camera_tis_node = ComposableNode(
        package='camera_tis',
        plugin='camera_tis::CameraTis',
        parameters=[params['camera_tis_node']],
        extra_arguments=[{'use_intra_process_comms': True}])

    params['rotate_image_node']['workers'] = 2
    rotate_image_node = ComposableNode(
        package='rotate_image',
        plugin='rotate_image::RotateImage',
        remappings=[('~/image', '/camera_tis_node/image')],
        parameters=[params['rotate_image_node']],
        extra_arguments=[{'use_intra_process_comms': True}])

    params['laser_line_center_node']['workers'] = 4
    laser_line_center_node = ComposableNode(
        package='laser_line_center',
        plugin='laser_line_center::LaserLineCenter',
        remappings=[('~/image', '/rotate_image_node/image_rotated')],
        parameters=[params['laser_line_center_node']],
        extra_arguments=[{'use_intra_process_comms': True}])

    params['laser_line_filter_node']['workers'] = 2
    laser_line_filter_node = ComposableNode(
        package='laser_line_filter',
        plugin='laser_line_filter::LaserLineFilter',
        remappings=[('~/line', '/laser_line_center_node/line')],
        parameters=[params['laser_line_filter_node']],
        extra_arguments=[{'use_intra_process_comms': True}])

    params['line_center_reconstruction_node']['workers'] = 2
    line_center_reconstruction_node = ComposableNode(
        package='line_center_reconstruction',
        plugin='line_center_reconstruction::LineCenterReconstruction',
        remappings=[('~/line', '/laser_line_filter_node/line_filtered')],
        parameters=[params['line_center_reconstruction_node']],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name='pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            camera_tis_node,
            rotate_image_node,
            laser_line_center_node,
            laser_line_filter_node,
            line_center_reconstruction_node])

    seam_tracking_node = Node(
        package='seam_tracking',
        executable='seam_tracking_node',
        remappings=[('~/pnts', '/line_center_reconstruction_node/pnts')],
        parameters=[params['seam_tracking_node']])

    modbus_node = Node(
        package='modbus',
        executable='modbus_node',
        parameters=[params['modbus_node']])

    gpio_raspberry_node = Node(
        package='gpio_raspberry',
        executable='gpio_raspberry_node',
        parameters=[params['gpio_raspberry_node']])

    config_tis_node = Node(
        package='config_tis',
        executable='config_tis_node',
        on_exit=launch.actions.Shutdown())

    return launch.LaunchDescription([
        container,
        seam_tracking_node,
        modbus_node,
        gpio_raspberry_node,
        config_tis_node])
