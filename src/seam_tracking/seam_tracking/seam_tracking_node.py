import rclpy

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
from shared_interfaces.srv import GetCode
from shared_interfaces.srv import SetCode
from shared_interfaces.srv import GetCodes
from shared_interfaces.srv import SetCodes
from shared_interfaces.srv import CountCodes

from .codes import Codes
import ros2_numpy as rnp
import numpy as np

# def Calculate(x, y, num, delta):
#     index = y.index(max(y))
#     if index - num < 0 and index + num > len(y):
#         return None, None
#     b1, m1 = polyfit(x[index - num : index - delta], y[index - num : index - delta], 1)
#     b2, m2 = polyfit(x[index + delta : index + num], y[index + delta : index + num], 1)
#     try:
#         px = (b2 - b1) / (m2 - m1)
#         py = px * m1 + b1
#     except:
#         return None, None
#     else:
#         return px, py

# def Convolve(x, y, k = 10, t = 5):
#     dy = [0.] * len(y)
#     for i in range(k, len(y) - k):
#         for j in range(-k, k + 1):
#             dy[i] += y[i] - y[i + j]
#     v = min(dy)
#     return dy.index(v) if v < -t else None

# def Segment(x, y, dx, dy, count = 1):
#     segX, segY = [], []
#     # if not x or not y:
#     #     return segX, segY
    
#     L = list(zip(x, y))

#     while(L):
#         (X, Y), *L = L
#         tmpX, tmpY = [X], [Y]
#         while(L):
#             if abs(X - L[0][0]) > dx or abs(Y - L[0][1]) > dy:
#                 break
#             else:
#                 (X, Y), *L = L
#                 tmpX.append(X)
#                 tmpY.append(Y)
#         if len(tmpX) > count:
#             segX.append(tmpX)
#             segY.append(tmpY)
    
#     return segX, segY

# def Candidates(segX, segY):
#     cx, cy = [], []
#     for lx, ly in zip(segX, segY):
#         cx.extend([lx[0], lx[-1]])
#         cy.extend([ly[0], ly[-1]])

#     return cx, cy

# def Pick(cx, cy, index):
#     if index < len(cx) or index < 0:
#         return cx[index], cy[index]
#     else:
#         return None, None

# def Exe(x, y, dx, dy, count = 1):
#     segX, segY = Segment(x, y, dx, dy, count)
#     return Candidates(segX, segY)


class SeamTracking(Node):


    def __init__(self):
        Node.__init__(self, 'seam_tracking_node')
        # self.pnts = [(None, None) for i in range(3)]
        self.declare_parameter('task')
        task = self.get_parameter('task')
        self.codes = Codes(id=task.get_parameter_value().integer_value)

        self.declare_parameter('delta_x')
        delta_x = self.get_parameter('delta_x')
        self.delta_x = delta_x.get_parameter_value().double_value

        self.declare_parameter('delta_y')
        delta_y = self.get_parameter('delta_y')
        self.delta_y = delta_y.get_parameter_value().double_value

        self.error = ''

        qos = rclpy.qos.qos_profile_sensor_data
        self.pub = self.create_publisher(PointCloud2, '~/seam', qos)
        self.sub = self.create_subscription(PointCloud2, '~/pnts', self._cb, qos)

        self.srv_get_code = self.create_service(GetCode, '~/get_code', self._cb_get_code)
        self.srv_set_code = self.create_service(SetCode, '~/set_code', self._cb_set_code)
        self.srv_get_codes = self.create_service(GetCodes, '~/get_codes', self._cb_get_codes)
        self.srv_set_codes = self.create_service(SetCodes, '~/set_codes', self._cb_set_codes)

        self.srv_count_codes = self.create_service(CountCodes, '~/count_codes', self._cb_count_codes)

        self.add_on_set_parameters_callback(self._cb_parameters)
        self.get_logger().info('Initialized successfully')

    def __del__(self):
        self.get_logger().info('Destroyed successfully')

    # def _append_pnts(self, ret, *, dx = 1., dy = 1.):
    #     self.pnts.append(ret)
    #     self.pnts.pop(0)
    #     for i in range(2):
    #         u0, v0 = self.pnts[i]
    #         u1, v1 = self.pnts[i + 1]
    #         if u0 == None or u1 == None or v0 == None or v1 == None:
    #             return None, None
    #         if abs(u1 - u0) > dx or abs(v1 - v0) > dy:
    #             return None, None
    #     return self.pnts[-1]

    def _cb_parameters(self, params):
        result = SetParametersResult()
        result.successful = True
        for p in params:
            if p.name == 'task':
                try:
                    self.codes.reload(id=p.get_parameter_value().integer_value)
                except Exception as e:
                    result.successful = False
                    result.reason = str(e)
            if p.name == 'delta_x':
                self.delta_x = p.get_parameter_value().double_value
            if p.name == 'delta_y':
                self.delta_y = p.get_parameter_value().double_value
        return result

    def _cb_get_code(self, request, response):
        try:
            id = None if request.index < 0 else request.index
            response.code = self.codes.get_code(id = id)
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_set_code(self, request, response):
        try:
            id = None if request.index < 0 else request.index
            self.codes.set_code(request.code, id = request.index)
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_get_codes(self, request, response):
        try:
            response.codes, response.index = self.codes.dumps()
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_set_codes(self, request, response):
        try:
            self.codes.loads(request.codes)
            self.codes.dump()
            self.codes.reload()
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb_count_codes(self, request, response):
        try:
            response.num = len(self.codes)
        except Exception as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def _cb(self, msg: PointCloud2):
        ret = PointCloud2()
        ret.header = msg.header

        if msg.data:
            d = rnp.numpify(msg)
            x = d['x'].tolist()
            y = d['y'].tolist()
            u = d['u'].tolist()
            v = d['v'].tolist()
            try:
                x, y = self.codes(x, y, u, v)
                x = [v + self.delta_x for v in x]
                y = [v + self.delta_y for v in y]
                d = np.array(list(zip(x, y)), dtype=[('x', np.float32), ('y', np.float32)])
                if d.size > 0:
                    ret = rnp.msgify(PointCloud2, d)
                    ret.header = msg.header
            except Exception as e:
                if self.error != str(e):
                    self.get_logger().warn(str(e))
                    self.error = str(e)
        self.pub.publish(ret)

def main(args=None):
    rclpy.init(args=args)

    sm = SeamTracking()

    try:
        rclpy.spin(sm)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        sm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
