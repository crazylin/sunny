from threading import Thread
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RosThread(Thread):
    """Ros subscription run in thread."""

    def __init__(self, node_name, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._sub = {}
        self._cli = {}
        
        if rclpy.ok() == False:
            rclpy.init()

        self._node = rclpy.create_node(node_name=node_name)

    def run(self):
        try:
            rclpy.spin(self._node)
        except KeyboardInterrupt:
            pass
        except ExternalShutdownException:
            pass
        finally:
            self._node.destroy_node()
            rclpy.try_shutdown()

        print('End ros thread cleanly')

    def create_subscription(self, sub_name, *args, **kwargs):
        if sub_name in self._sub:
            self._node.destroy_subscription(self._sub[sub_name])
        self._sub[sub_name] = self._node.create_subscription(*args, **kwargs)

    def create_client(self, cli_name, *args, **kwargs):
        if cli_name in self._cli:
            self._node.destroy_client(self._cli[cli_name])
        self._cli[cli_name] = self._node.create_client(*args, **kwargs)

    def remove_subscription(self, sub_name):
        if sub_name in self._sub:
            del self._sub[sub_name]

    def remove_client(self, cli_name):
        if cli_name in self._cli:
            del self._cli[cli_name]
