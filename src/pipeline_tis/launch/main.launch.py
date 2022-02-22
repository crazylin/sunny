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
    configFile1 = os.path.join(
        get_package_share_directory('camera_tis'),
        'config',
        'params.yaml')

    with open(configFile1, 'r') as file:
        configParams1 = yaml.safe_load(file)['camera_tis_node']['ros__parameters']

    node1 = ComposableNode(
        package='camera_tis',
        plugin='camera_tis::CameraTis',
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
        remappings=[('~/image', '/camera_tis_node/image')],
        parameters=[configParams2],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile3 = os.path.join(
        get_package_share_directory('branch_image'),
        'config',
        'params.yaml')

    with open(configFile3, 'r') as file:
        configParams3 = yaml.safe_load(file)['branch_image_node']['ros__parameters']

    node3 = ComposableNode(
        package='branch_image',
        plugin='branch_image::BranchImage',
        remappings=[('~/image', '/rotate_image_node/image_rotated')],
        parameters=[configParams3],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile4 = os.path.join(
        get_package_share_directory('laser_line_center'),
        'config',
        'params.yaml')

    with open(configFile4, 'r') as file:
        handle = yaml.safe_load(file)
        configParams4 = handle['laser_line_center_node']['ros__parameters']

    node4_l = ComposableNode(
        package='laser_line_center',
        plugin='laser_line_center::LaserLineCenter',
        name='laser_line_center_node_l',
        remappings=[
            ('~/image', '/branch_image_node/image_l'),
            ('~/line', 'laser_line_center_node/line')],
        parameters=[configParams4],
        extra_arguments=[{'use_intra_process_comms': True}])

    node4_r = ComposableNode(
        package='laser_line_center',
        plugin='laser_line_center::LaserLineCenter',
        name='laser_line_center_node_r',
        remappings=[
            ('~/image', '/branch_image_node/image_r'),
            ('~/line', 'laser_line_center_node/line')],
        parameters=[configParams4],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile5 = os.path.join(
        get_package_share_directory('line_center_reconstruction'),
        'config',
        'params.yaml')

    with open(configFile5, 'r') as file:
        handle = yaml.safe_load(file)
        configParams5 = handle['line_center_reconstruction_node']['ros__parameters']

    node5 = ComposableNode(
        package='line_center_reconstruction',
        plugin='line_center_reconstruction::LineCenterReconstruction',
        remappings=[('~/line', '/laser_line_center_node/line')],
        parameters=[configParams5],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile6 = os.path.join(
        get_package_share_directory('modbus'),
        'config',
        'params.yaml')

    with open(configFile6, 'r') as file:
        handle = yaml.safe_load(file)
        configParams6 = handle['modbus_node']['ros__parameters']

    node6 = ComposableNode(
        package='modbus',
        plugin='modbus::Modbus',
        remappings=[('~/coord', '/seam_tracking_node/coord')],
        parameters=[configParams6],
        extra_arguments=[{'use_intra_process_comms': True}])

    configFile7 = os.path.join(
        get_package_share_directory('gpio_raspberry'),
        'config',
        'params.yaml')

    with open(configFile7, 'r') as file:
        handle = yaml.safe_load(file)
        configParams7 = handle['gpio_raspberry_node']['ros__parameters']

    node7 = ComposableNode(
        package='gpio_raspberry',
        plugin='gpio_raspberry::GpioRaspberry',
        parameters=[configParams7],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
        name='pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[node1, node2, node3, node4_l, node4_r, node5, node6, node7],
        output='screen')

    node8 = Node(
        package='seam_tracking',
        executable='main',
        remappings=[('~/pnts', '/line_center_reconstruction_node/pnts')])

    return launch.LaunchDescription([container, node8])
