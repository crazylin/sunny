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

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
# from shared_interfaces.srv import GetCode
# from shared_interfaces.srv import SetCode
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue, Log


def from_parameter_value(p: ParameterValue):
    if p.type == ParameterType.PARAMETER_NOT_SET:
        return None
    elif p.type == ParameterType.PARAMETER_BOOL:
        return p.bool_value
    elif p.type == ParameterType.PARAMETER_DOUBLE:
        return p.double_value
    elif p.type == ParameterType.PARAMETER_INTEGER:
        return p.integer_value
    elif p.type == ParameterType.PARAMETER_STRING:
        return p.string_value
    elif p.type == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(p.bool_array_value)
    elif p.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(p.double_array_value)
    elif p.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(p.integer_array_value)
    elif p.type == ParameterType.PARAMETER_STRING_ARRAY:
        return list(p.string_array_value)
    else:
        return None


def to_parameter_value(v):
    if type(v) is int:
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=v)
    elif type(v) is float:
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=v)
    elif type(v) is bool:
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
    elif type(v) is str:
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
    elif type(v) is list:
        if type(v[0]) is int:
            return ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=v)
        elif type(v[0]) is float:
            return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=v)
        elif type(v[0]) is bool:
            return ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY, bool_array_value=v)
        elif type(v[0]) is str:
            return ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=v)
    else:
        return None


class RosNode(Node):
    """Ros node."""

    def __init__(self, params: dict):
        super().__init__('seam_tracking_gui')

        self._pub = {}
        self._sub = {}
        self._cli = {}
        self._cli_get = {}
        self._cli_set = {}

        for k in params:
            self._cli_get[k] = self.create_client(GetParameters, f'/{k}/get_parameters')
            self._cli_set[k] = self.create_client(SetParameters, f'/{k}/set_parameters')

        self._create_publisher('config', String, '/config_tis_node/config', 10)
        # self._create_client('get_code', GetCode, '/seam_tracking_node/get_code')
        # self._create_client('set_code', SetCode, '/seam_tracking_node/set_code')

    def pub_config(self, msg: str):
        s = String()
        s.data = msg
        self._pub['config'].publish(s)

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

    def sub_log(self, cb):
        self._create_subscription(
            'log',
            Log,
            '/rosout',
            cb,
            10)

    # def get_code(self, index: int):
    #     cli = self._cli['get_code']
    #     if cli.service_is_ready():
    #         request = GetCode.Request()
    #         request.index = index
    #         return cli.call_async(request)
    #     else:
    #         return None

    # def set_code(self, index: int, code: str):
    #     cli = self._cli['set_code']
    #     if cli.service_is_ready():
    #         request = SetCode.Request()
    #         request.index = index
    #         request.code = code
    #         return cli.call_async(request)
    #     else:
    #         return None

    def get_params(self, name: str, params: list):
        cli = self._cli_get[name]
        if cli.service_is_ready():
            request = GetParameters.Request()
            request.names = params
            return cli.call_async(request)
        else:
            return None

    def set_params(self, name: str, d: dict):
        cli = self._cli_set[name]
        if cli.service_is_ready():
            request = SetParameters.Request()
            for k, v in d.items():
                value = to_parameter_value(v)
                request.parameters.append(Parameter(name=k, value=value))
            return cli.call_async(request)
        else:
            return None

    def _create_publisher(self, pub_name: str, *args, **kwargs):
        if pub_name in self._pub:
            self.destroy_publisher(self._pub[pub_name])
        self._pub[pub_name] = self.create_publisher(*args, **kwargs)

    def _create_subscription(self, sub_name: str, *args, **kwargs):
        if sub_name in self._sub:
            self.destroy_subscription(self._sub[sub_name])
        self._sub[sub_name] = self.create_subscription(*args, **kwargs)

    def _create_client(self, cli_name: str, *args, **kwargs):
        if cli_name in self._cli:
            self.destroy_client(self._cli[cli_name])
        self._cli[cli_name] = self.create_client(*args, **kwargs)

    def _remove_publisher(self, pub_name: str):
        if pub_name in self._pub:
            del self._pub[pub_name]

    def _remove_subscription(self, sub_name: str):
        if sub_name in self._sub:
            del self._sub[sub_name]

    def _remove_client(self, cli_name: str):
        if cli_name in self._cli:
            del self._cli[cli_name]
