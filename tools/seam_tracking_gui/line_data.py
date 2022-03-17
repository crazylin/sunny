from threading import Lock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

class LineData():
    """For line data."""

    def __init__(self):
        self._lock = Lock()
        self._u = []
        self._v = []
        self._f = None
        self._h = Header()

    def from_msg(self, msg: PointCloud2, *, u: str, v: str):
        with self._lock:
            if msg.data:
                d = rnp.numpify(msg)
                self._u = d[u].tolist()
                self._v = d[v].tolist()
            else:
                self._u = []
                self._v = []
            self._f = self._cal_fps(msg.header)
            self._h = msg.header

    def get(self):
        with self._lock:
            return self._u, self._v, self._h.frame_id, self._f

    def _cal_fps(self, h: Header):
        try:
            dt_sec = h.stamp.sec - self._h.stamp.sec
            dt_nano = h.stamp.nanosec - self._h.stamp.nanosec
            dt = dt_sec + dt_nano * 1e-9
            df = int(h.frame_id) - int(self._h.frame_id)
            return df / dt
        except:
            return None