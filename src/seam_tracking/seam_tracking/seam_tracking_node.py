"""
ROS node to locate seams from points, supports source code plugin.

A python ROS node to subscribe from upstream topic.
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

import socket
import time
import rclpy
import threading
import queue

from collections import deque
from rclpy.node import Node
# from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
# from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
# from shared_interfaces.srv import GetCode
# from shared_interfaces.srv import SetCode

from .codes import Codes
import ros2_numpy as rnp
import numpy as np


class SeamTracking(Node):
    """
    ROS node to locate seam and maintain plugins.

    A python ROS node to subscribe from upstream topic.
    Apply a serial of customizable plugins to locate seam.
    """

    def __init__(self):
        Node.__init__(self, 'seam_tracking_node')
        # self._dtype = [('x', np.float32), ('y', np.float32), ('i', np.float32)]
        self._deq = deque()

        self.declare_parameter('window_size', 10)
        self._ws = self.get_parameter('window_size').value

        self.declare_parameter('gap', 2)
        self._gap = self.get_parameter('gap').value

        self.declare_parameter('step', 2.)
        self._step = self.get_parameter('step').value

        self.declare_parameter('length', 5)
        self._length = self.get_parameter('length').value

        self.declare_parameter('enable', False)
        self._enable = self.get_parameter('enable').value

        self.declare_parameter('task', 0)
        self._task = self.get_parameter('task').value

        self.declare_parameter('delta_x', 0.)
        self._delta_x = self.get_parameter('delta_x').value

        self.declare_parameter('delta_y', 0.)
        self._delta_y = self.get_parameter('delta_y').value

        self.declare_parameter('codes', [''])
        self._codes = Codes(self.get_parameter('codes').value)

        try:
            self._codes.reload(self._task)
        except Exception as e:
            self.get_logger().error(str(e))
        # finally:
        #     self.set_parameters([Parameter('codes', Parameter.Type.STRING_ARRAY, self._codes)])

        self._error = ''

        self.pub = self.create_publisher(
            PointCloud2,
            '~/seam',
            rclpy.qos.qos_profile_sensor_data)
        self.sub = self.create_subscription(
            PointCloud2,
            '~/pnts',
            self._cb_sub,
            rclpy.qos.qos_profile_sensor_data)

        # self.srv_get_code = self.create_service(
        #     GetCode,
        #     '~/get_code',
        #     self._cb_get_code)
        # self.srv_set_code = self.create_service(
        #     SetCode,
        #     '~/set_code',
        #     self._cb_set_code)

        # self.srv_dump_codes = self.create_service(
        #     Trigger,
        #     '~/dump_codes',
        #     self._cb_dump_codes)
        # self.srv_load_codes = self.create_service(
        #     Trigger,
        #     '~/load_codes',
        #     self._cb_load_codes)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # for i in range(10):
        #     ret = self._sock.connect_ex(("127.0.0.1", 2345))
        #     if ret == 0:
        #         self._sock.settimeout(0.5)
        #         self.get_logger().info('Connect to local server successfully')
        #         break
        #     else:
        #         time.sleep(0.5)
        # else:
        #     # Failed to connect to server.
        #     self._sock = None
        #     self.get_logger().warn('Failed to connect to local server')
        self._q = queue.Queue(30)
        t = threading.Thread(target=self._socket, daemon=True)
        t.start()
        self.get_logger().info('Initialized successfully')

    def __del__(self):
        self.get_logger().info('Destroyed successfully')

    def _socket(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            while True:
                try:
                    s.connect(("127.0.0.1", 2345))
                    break
                except Exception:
                    time.sleep(5)

            self.get_logger().info('Socket connect successfully')
            while True:
                try:
                    f, b, u, v = self._q.get(block=True)
                    msg = self._modbus_msg(f, b, u, v)
                    s.sendall(msg)
                    s.recv(256)
                except Exception as e:
                    if self._error != str(e):
                        self.get_logger().error(str(e))
                        self._error = str(e)

    def _on_set_parameters(self, params):
        result = SetParametersResult()
        result.successful = True
        for p in params:
            if p.name == 'task':
                try:
                    self._task = p.value
                    self._codes.reload(self._task)
                except Exception as e:
                    self.get_logger().error(str(e))
            elif p.name == 'codes':
                try:
                    self._codes[:] = p.value
                    self._codes.reload(self._task)
                except Exception as e:
                    self.get_logger().error(str(e))
            elif p.name == 'delta_x':
                self._delta_x = p.value
            elif p.name == 'delta_y':
                self._delta_y = p.value
            elif p.name == 'window_size':
                self._ws = p.value
            elif p.name == 'gap':
                self._gap = p.value
            elif p.name == 'step':
                self._step = p.value
            elif p.name == 'length':
                self._length = p.value
            elif p.name == 'enable':
                self._enable = p.value
        return result

    # def _cb_get_code(self, request, response):
    #     try:
    #         response.code = self._codes[request.index]
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     else:
    #         response.success = True
    #     return response

    # def _cb_set_code(self, request, response):
    #     try:
    #         self._codes[request.index] = request.code
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     else:
    #         response.success = True
    #     return response

    # def _cb_dump_codes(self, request, response):
    #     try:
    #         self._codes.dump(self._file)
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     else:
    #         response.success = True
    #     return response

    # def _cb_load_codes(self, request, response):
    #     try:
    #         self._codes.load(self._file)
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     else:
    #         response.success = True
    #     return response

    def _cb_sub(self, msg: PointCloud2):
        """
        Subscription callback.

        The call takes a ROS point cloud message.
        It uses the customized codes to process the data.
        The output data is further processed with filter, offset and removal nan.
        Finally, a ROS point cloud message is published.

        :param msg: ROS point cloud message.
        :type msg: PointCloud2
        """
        ret = PointCloud2()
        ret.header = msg.header

        valid = False
        u = 0.
        v = 0.
        if msg.data:
            try:
                pnts_xyi = rnp.numpify(msg)
                pnts_xyi = self._codes(pnts_xyi)
                pnts_xyi = self._filter(pnts_xyi)
                pnts_xyi = self._offset(pnts_xyi)
                pnts_xyi = self._notnan(pnts_xyi)
                if pnts_xyi[0][2] == -1:
                    valid = True
                    u = float(pnts_xyi[0][0])
                    v = float(pnts_xyi[0][1])
                ret = rnp.msgify(PointCloud2, pnts_xyi)
                ret.header = msg.header
            except Exception as e:
                if self._error != str(e):
                    self.get_logger().error(str(e))
                    self._error = str(e)
        try:
            self._q.put((ret.header.frame_id, valid, u, v), block=False)
        except Exception as e:
            if self._error != str(e):
                self.get_logger().error(str(e))
                self._error = str(e)
        self.pub.publish(ret)

    def _filter(self, r: np.ndarray):
        if len(r) and r[0][2] == -1:
            self._deq.appendleft(r[0][1])
        else:
            self._deq.appendleft(None)
            return r

        while len(self._deq) > self._ws:
            self._deq.pop()

        if not self._enable:
            return r

        i = 0
        j = 1
        while j < len(self._deq):
            if j - i > self._gap or i >= self._length:
                break
            if self._deq[j] is None:
                j += 1
                continue
            du = abs(self._deq[j][0] - self._deq[i][0]) / (j - i)
            dv = abs(self._deq[j][1] - self._deq[i][1]) / (j - i)
            if du > self._step or dv > self._step:
                j += 1
                continue
            i = j
            j += 1

        if i >= self._length:
            return r
        else:
            r[0][2] = -99
            return r

    def _offset(self, r: np.ndarray):
        """
        Offset the picked point in milli meter.

        :param r: Point cloud in nunpy.
        :type r: np.ndarray
        :return: The input is modified in place and returned.
        :rtype: np.ndarray
        """
        if r[0][2] == -1:
            r[0][0] += self._delta_x
            r[0][1] += self._delta_y
        return r

    def _notnan(self, r: np.ndarray):
        mask = ~np.isnan(r['x'])
        return r[mask]

    def _modbus_msg(self, id: str, valid: bool, u: float, v: float):
        id = int(id) % 0x10000
        # Transaction identifier
        s = id.to_bytes(2, 'big')
        # Protocol identifier 2, Length field 2, Unit identifier 1, Function code 1
        s += bytes([0x00, 0x00, 0x00, 0x0d, 0x01, 0x10])
        # Start address 2, number of registers, number of bytes
        s += bytes([0x00, 0x02, 0x00, 0x03, 0x06])

        b = bytes([0x00, 0xff]) if valid else bytes([0x00, 0x00])

        try:
            t = bytes()
            t += round(u * 100).to_bytes(2, 'big')
            t += round(v * 100).to_bytes(2, 'big')
        except Exception:
            s += bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        else:
            s += b + t

        return s


def main(args=None):
    rclpy.init(args=args)

    seam_tracking = SeamTracking()

    try:
        rclpy.spin(seam_tracking)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        seam_tracking.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
