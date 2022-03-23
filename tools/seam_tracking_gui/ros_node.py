from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
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
        self._param_exposure = None
        self._param_task = None
        self._param_delta_x = None
        self._param_delta_y = None

        self._sub = {}
        self._cli = {}

        self._create_client('get_code', GetCode, '/seam_tracking_node/get_code')
        self._create_client('set_code', SetCode, '/seam_tracking_node/set_code')
        self._create_client('get_codes', GetCodes, '/seam_tracking_node/get_codes')
        self._create_client('set_codes', SetCodes, '/seam_tracking_node/set_codes')

        self._create_client('get_power', GetParameters, '/camera_tis_node/get_parameters')
        self._create_client('set_power', SetParameters, '/camera_tis_node/set_parameters')

        self._create_client('get_laser', GetParameters, '/gpio_raspberry_node/get_parameters')
        self._create_client('set_laser', SetParameters, '/gpio_raspberry_node/set_parameters')

        self._create_client('get_exposure', GetParameters, '/camera_tis_node/get_parameters')
        self._create_client('set_exposure', SetParameters, '/camera_tis_node/set_parameters')

        self._create_client('get_task', GetParameters, '/seam_tracking_node/get_parameters')
        self._create_client('set_task', SetParameters, '/seam_tracking_node/set_parameters')

        self._create_client('get_delta', GetParameters, '/seam_tracking_node/get_parameters')
        self._create_client('set_delta', SetParameters, '/seam_tracking_node/set_parameters')

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

    def get_power(self):
        cli = self._cli['get_power']
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = ['power']
            return cli.call_async(request)
        else:
            return None

    def set_power(self, power: bool):
        cli = self._cli['set_power']
        if cli.service_is_ready():
            value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=power)
            request = SetParameters.Request()
            request.parameters = [Parameter(name='power', value=value)]
            return cli.call_async(request)
        else:
            return None

    def get_laser(self):
        cli = self._cli['get_laser']
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = ['laser']
            return cli.call_async(request)
        else:
            return None

    def set_laser(self, laser: bool):
        cli = self._cli['set_laser']
        if cli.service_is_ready():
            value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=laser)
            request = SetParameters.Request()
            request.parameters = [Parameter(name='laser', value=value)]
            return cli.call_async(request)
        else:
            return None

    def _get_exposure_done(self, future):
        try:
            res = future.result()
            self._param_exposure = res.values[0].integer_value
        except Exception as e:
            self.get_logger().error(str(e))

    def get_exposure(self):
        cli = self._cli['get_exposure']
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = ['exposure_time']
            f = cli.call_async(request)
            f.add_done_callback(self._get_exposure_done)
        else:
            self.get_logger().info('Service is not ready!')

    def set_exposure(self, exposure):
        cli = self._cli['set_exposure']
        if cli.service_is_ready():
            value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=exposure)
            request = SetParameters.Request()
            request.parameters = [Parameter(name='exposure_time', value=value)]
            return cli.call_async(request)
        else:
            return None

    def _get_task_done(self, future):
        try:
            res = future.result()
            self._param_task = res.values[0].integer_value
        except Exception as e:
            self.get_logger().error(str(e))

    def get_task(self):
        cli = self._cli['get_task']
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = ['task']
            f = cli.call_async(request)
            f.add_done_callback(self._get_task_done)
        else:
            self.get_logger().info('Service is not ready!')

    def set_task(self, task):
        cli = self._cli['set_task']
        if cli.service_is_ready():
            value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=task)
            request = SetParameters.Request()
            request.parameters = [Parameter(name='task', value=value)]
            return cli.call_async(request)
        else:
            return None

    def _get_delta_done(self, future):
        try:
            res = future.result()
            self._param_delta_x, self._param_delta_y = res.values[0].double_value, res.values[1].double_value
        except Exception as e:
            self.get_logger().error(str(e))

    def get_delta(self):
        cli = self._cli['get_delta']
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = ['delta_x', 'delta_y']
            f = cli.call_async(request)
            f.add_done_callback(self._get_delta_done)
        else:
            self.get_logger().info('Service is not ready!')

    def set_delta(self, delta_x, delta_y):
        cli = self._cli['set_delta']
        if cli.service_is_ready():
            value_x = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=delta_x)
            value_y = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=delta_y)
            request = SetParameters.Request()
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