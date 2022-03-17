import rclpy
from threading import Thread
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from shared_interfaces.msg import ModbusCoord
from shared_interfaces.srv import GetCode
from shared_interfaces.srv import SetCode
from shared_interfaces.srv import GetCodes
from shared_interfaces.srv import SetCodes
from shared_interfaces.srv import CountCodes
from shared_interfaces.srv import SelectCode


class RosNode(Node):
    """Ros node."""

    def __init__(self):
        super().__init__('seam_tracking_gui')
        self._sub = {}
        self._cli = {}
        self._create_client('get_code', GetCode, '/seam_tracking_node/get_code')
        self._create_client('set_code', SetCode, '/seam_tracking_node/set_code')
        self._create_client('get_codes', GetCodes, '/seam_tracking_node/get_codes')
        self._create_client('set_codes', SetCodes, '/seam_tracking_node/set_codes')

    def spin(self):
        rclpy.spin(self)
        # try:
        #     rclpy.spin(self)
        # except KeyboardInterrupt:
        #     print('key')
        # except ExternalShutdownException:
        #     print('external')
        # else:
        #     print('clean')
        # finally:
        #     rclpy.shutdown()

    def sub_line(self, cb):
        qos = qos_profile_sensor_data
        qos.depth = 1
        self._create_subscription(
            'line',
            PointCloud2,
            '/line_center_reconstruction_node/pnts',
            cb,
            qos)

    def sub_pick(self, cb):
        self._create_subscription(
            'pick',
            ModbusCoord,
            '/seam_tracking_node/coord',
            cb,
            10)

    def get_code(self, *, id = -1):
        cli = self._cli['get_code']
        if cli.service_is_ready():
            request = GetCode.Request()
            request.index = id
            return cli.call_async(request)
        else:
            return None

    def set_code(self, code, *, id = -1):
        cli = self._cli['set_code']
        if cli.service_is_ready():
            request = SetCode.Request()
            request.index = id
            request.code = code
            return cli.call_async(request)
        else:
            return None

    def get_codes(self):
        cli = self._cli['get_codes']
        if cli.service_is_ready():
            request = GetCodes.Request()
            return cli.call_async(request)
        else:
            return None

    def set_codes(self, codes):
        cli = self._cli['set_codes']
        if cli.service_is_ready():
            request = SetCodes.Request()
            request.codes = codes
            return cli.call_async(request)
        else:
            return None

    def _create_subscription(self, sub_name, *args, **kwargs):
        if sub_name in self._sub:
            self.destroy_subscription(self._sub[sub_name])
        self._sub[sub_name] = self.create_subscription(*args, **kwargs)

    def _create_client(self, cli_name, *args, **kwargs):
        if cli_name in self._cli:
            self.destroy_client(self._cli[cli_name])
        self._cli[cli_name] = self.create_client(*args, **kwargs)

    def _remove_subscription(self, sub_name):
        if sub_name in self._sub:
            del self._sub[sub_name]

    def _remove_client(self, cli_name):
        if cli_name in self._cli:
            del self._cli[cli_name]
