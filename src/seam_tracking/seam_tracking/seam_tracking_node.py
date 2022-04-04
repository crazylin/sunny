import os
import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from shared_interfaces.srv import GetCode
from shared_interfaces.srv import SetCode

from .codes import Codes
import ros2_numpy as rnp
import numpy as np

class SeamTracking(Node):

    def __init__(self):
        Node.__init__(self, 'seam_tracking_node')

        self.declare_parameter('task', 0)
        self._task = self.get_parameter('task').value

        self.declare_parameter('codes', [''], ignore_override=True)
        self._file = os.path.join(os.path.dirname(__file__), 'codes.json')
        self._codes = Codes()
        try:
            self._codes.load(self._file)
            self._codes.reload(self._task)
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            self.set_parameters([Parameter('codes', Parameter.Type.STRING_ARRAY, self._codes)])

        self.declare_parameter('delta_x', 0.)
        self._delta_x = self.get_parameter('delta_x').value

        self.declare_parameter('delta_y', 0.)
        self._delta_y = self.get_parameter('delta_y').value

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

        self.srv_get_code = self.create_service(
            GetCode,
            '~/get_code',
            self._cb_get_code)
        self.srv_set_code = self.create_service(
            SetCode,
            '~/set_code',
            self._cb_set_code)

        self.srv_dump_codes = self.create_service(
            Trigger,
            '~/dump_codes',
            self._cb_dump_codes)
        self.srv_load_codes = self.create_service(
            Trigger,
            '~/load_codes',
            self._cb_load_codes)

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.get_logger().info('Initialized successfully')

    def __del__(self):
        self.get_logger().info('Destroyed successfully')

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
        return result

    def _cb_get_code(self, request, response):
        try:
            response.code = self._codes[request.index]
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_set_code(self, request, response):
        try:
            self._codes[request.index] = request.code
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_dump_codes(self, request, response):
        try:
            self._codes.dump(self._file)
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_load_codes(self, request, response):
        try:
            self._codes.load(self._file)
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_sub(self, msg: PointCloud2):
        ret = PointCloud2()
        ret.header = msg.header
        if len(msg.data):
            try:
                d = rnp.numpify(msg)
                r = self._codes(d)
                if r is not None and len(r):
                    r['x'][0] += self._delta_x
                    r['y'][0] += self._delta_y
                    ret = rnp.msgify(PointCloud2, r)
                    ret.header = msg.header
            except Exception as e:
                if self._error != str(e):
                    self.get_logger().error(str(e))
                    self._error = str(e)
        self.pub.publish(ret)

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
