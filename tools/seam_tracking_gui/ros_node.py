from point_data import PointData
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
from shared_interfaces.srv import GetCode
from shared_interfaces.srv import SetCode
from shared_interfaces.srv import GetCodes
from shared_interfaces.srv import SetCodes
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class RosNode(Node):
    """Ros node."""

    def __init__(self):
        super().__init__('seam_tracking_gui')
        self._sub = {}
        self._cli = {}
        self._create_client('laser_on', Trigger, '/gpio_raspberry_node/high')
        self._create_client('laser_off', Trigger, '/gpio_raspberry_node/low')
        self._create_client('camera_on', Trigger, '/camera_tis_node/start')
        self._create_client('camera_off', Trigger, '/camera_tis_node/stop')
        self._create_client('get_code', GetCode, '/seam_tracking_node/get_code')
        self._create_client('set_code', SetCode, '/seam_tracking_node/set_code')
        self._create_client('get_codes', GetCodes, '/seam_tracking_node/get_codes')
        self._create_client('set_codes', SetCodes, '/seam_tracking_node/set_codes')

        self._create_client('set_exposure', SetParameters, '/camera_tis_node/set_parameters')
        self._create_client('set_task', SetParameters, '/seam_tracking_node/set_parameters')
        self._create_client('set_delta', SetParametersAtomically, '/seam_tracking_node/set_parameters_atomically')

    # def sub_pnts(self, cb):
    #     qos = qos_profile_sensor_data
    #     qos.depth = 1
    #     self._create_subscription(
    #         'pnts',
    #         PointCloud2,
    #         '/line_center_reconstruction_node/pnts',
    #         cb,
    #         qos)

    def sub_seam(self, cb):
        qos = qos_profile_sensor_data
        qos.depth = 1
        self._create_subscription(
            'seam',
            PointCloud2,
            '/seam_tracking_node/seam',
            cb,
            qos)

    def laser_on(self):
        cli = self._cli['laser_on']
        if cli.service_is_ready():
            request = Trigger.Request()
            return cli.call_async(request)
        else:
            return None

    def laser_off(self):
        cli = self._cli['laser_off']
        if cli.service_is_ready():
            request = Trigger.Request()
            return cli.call_async(request)
        else:
            return None

    def camera_on(self):
        cli = self._cli['camera_on']
        if cli.service_is_ready():
            request = Trigger.Request()
            return cli.call_async(request)
        else:
            return None

    def camera_off(self):
        cli = self._cli['camera_off']
        if cli.service_is_ready():
            request = Trigger.Request()
            return cli.call_async(request)
        else:
            return None

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

    def set_exposure(self, exposure):
        cli = self._cli['set_exposure']
        if cli.service_is_ready():
            value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=exposure)
            request = SetParameters.Request()
            request.parameters = [Parameter(name='exposure_time', value=value)]
            return cli.call_async(request)
        else:
            return None

    def set_task(self, task):
        cli = self._cli['set_task']
        if cli.service_is_ready():
            value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=task)
            request = SetParameters.Request()
            request.parameters = [Parameter(name='task', value=value)]
            return cli.call_async(request)
        else:
            return None

    def set_delta(self, delta_x, delta_y):
        cli = self._cli['set_delta']
        if cli.service_is_ready():
            value_x = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=delta_x)
            value_y = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=delta_y)
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter(name='delta_x', value=value_x), Parameter(name='delta_y', value=value_y)]
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