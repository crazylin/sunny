from threading import Lock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

class PointData():
    """For point(x, y) data."""

    def __init__(self):
        self._lock = Lock()
        self._x = []
        self._y = []
        self._z = []
        self._f = None
        self._h = Header()

    def from_msg(self, msg: PointCloud2):
        with self._lock:
            if msg.data:
                d = rnp.numpify(msg)
                self._x = d['x'].tolist()
                self._y = d['y'].tolist()
                self._z = d['z'].tolist()
            else:
                self._x = []
                self._y = []
                self._z = []
            self._f = self._cal_fps(msg.header)
            self._h = msg.header

    def get(self):
        with self._lock:
            return self._x, self._y, self._z, self._h.frame_id, self._f

    def _cal_fps(self, h: Header):
        try:
            dt_sec = h.stamp.sec - self._h.stamp.sec
            dt_nano = h.stamp.nanosec - self._h.stamp.nanosec
            dt = dt_sec + dt_nano * 1e-9
            df = int(h.frame_id) - int(self._h.frame_id)
            return df / dt
        except:
            return None