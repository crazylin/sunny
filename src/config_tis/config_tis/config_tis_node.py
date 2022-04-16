import os
import sys
import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class ConfigTis(Node):

    def __init__(self):
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
        self.get_logger().info('Destroyed successfully')

    def _cb_sub(self, msg: String):
        if msg.data == 'restart':
            sys.exit(0)
        with open(self._file, 'w') as fp:
            fp.write(msg.data)

def main(args=None):
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