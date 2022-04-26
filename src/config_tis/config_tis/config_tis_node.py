"""
The one config node for almost all parameters in the pipeline.

The config node use two layers of paramter file:
The underlay `.params.yaml` exists as read only or backup or default file.
It provides default values and meaningful value if the overlay failed.

The overlay `params.yaml` is to save most recently user modifications.
It is read and updated to the underlay.

If the overlay is empty, it also means use everything in the underlay,
which is everything in its defaults.

This node also accept a special message: `'restart'` which causes the system to restart.
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
import sys
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class ConfigTis(Node):
    """A config inherited from ROS node."""

    def __init__(self):
        """
        Initialize itself.

        Specify itself a name. Construct a file path to overlay: params.yaml.
        The underlay: .params.yml is read only, used as backup and defaults.
        Initialize a subscription.
        Print success if all done.
        """
        Node.__init__(self, 'config_tis_node')
        self._file = os.path.join(
            get_package_share_directory('config_tis'),
            'config',
            'params.yaml')
        self._sub = self.create_subscription(
            String,
            '~/config',
            self._cb_sub,
            10
        )
        self.get_logger().info('Initialized successfully')

    def __del__(self):
        """Print success if all done."""
        self.get_logger().info('Destroyed successfully')

    def _cb_sub(self, msg: String):
        """
        Subscription callback.

        Special msg with literal 'restart' instruct the pipeline to restart itself.
        Empty msg effectively resorts to defaults.

        :param msg: A String containing serialized yaml object.
        :type msg: String
        """
        if msg.data == 'restart':
            sys.exit(0)
        with open(self._file, 'w') as fp:
            fp.write(msg.data)


def main(args=None):
    """Spin the node."""
    rclpy.init(args=args)

    config_tis = ConfigTis()

    try:
        rclpy.spin(config_tis)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        config_tis.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
