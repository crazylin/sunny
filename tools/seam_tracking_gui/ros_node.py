from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from shared_interfaces.srv import GetCode
from shared_interfaces.srv import SetCode
from shared_interfaces.srv import GetCodes
from shared_interfaces.srv import SetCodes
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


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

        self._create_client('camera_get', GetParameters, '/camera_tis_node/get_parameters')
        self._create_client('camera_set', SetParameters, '/camera_tis_node/set_parameters')

        self._create_client('gpio_get', GetParameters, '/gpio_raspberry_node/get_parameters')
        self._create_client('gpio_set', SetParameters, '/gpio_raspberry_node/set_parameters')

        self._create_client('seam_get', GetParameters, '/seam_tracking_node/get_parameters')
        self._create_client('seam_set', SetParameters, '/seam_tracking_node/set_parameters')

    def sub_pnts(self, cb):
        qos = qos_profile_sensor_data
        qos.depth = 1
        self._create_subscription(
            'pnts',
            PointCloud2,
            '/line_center_reconstruction_node/pnts',
            cb,
            qos)

    def sub_seam(self, cb):
        qos = qos_profile_sensor_data
        qos.depth = 1
        self._create_subscription(
            'seam',
            PointCloud2,
            '/seam_tracking_node/seam',
            cb,
            qos)

    def get_code(self, *, id: int = -1):
        cli = self._cli['get_code']
        if cli.service_is_ready():
            request = GetCode.Request()
            request.index = id
            return cli.call_async(request)
        else:
            return None

    def set_code(self, code, *, id: int = -1):
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

    def set_codes(self, codes: str):
        cli = self._cli['set_codes']
        if cli.service_is_ready():
            request = SetCodes.Request()
            request.codes = codes
            return cli.call_async(request)
        else:
            return None

    def camera_get(self, params: list):
        return self._get_params('camera_get', params)

    def camera_set(self, d: dict):
        return self._set_params('camera_set', d)

    def gpio_get(self, params: list):
        return self._get_params('gpio_get', params)

    def gpio_set(self, d: dict):
        return self._set_params('gpio_set', d)

    def seam_get(self, params: list):
        return self._get_params('seam_get', params)

    def seam_set(self, d: dict):
        return self._set_params('seam_set', d)

    def _get_params(self, name: str, params: list):
        cli = self._cli[name]
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = params
            return cli.call_async(request)
        else:
            return None

    def _set_params(self, name: str, d: dict):
        cli = self._cli[name]
        if cli.service_is_ready():
            request = SetParameters.Request()
            for k, v in d.items():
                if type(v) is int:
                    value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
                elif type(v) is float:
                    value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
                elif type(v) is bool:
                    value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
                else:
                    continue
                request.parameters.append(Parameter(name=k, value=value))
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
