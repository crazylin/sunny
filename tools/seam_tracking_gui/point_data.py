from collections import deque
from threading import Lock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

class PntsData():
    """For point(x, y) data."""

    def __init__(self):
        self._lock = Lock()
        self._x = []
        self._y = []

    def from_msg(self, msg: PointCloud2):
        with self._lock:
            if len(msg.data):
                d = rnp.numpify(msg)
                self._x = d['x'].tolist()
                self._y = d['y'].tolist()
            else:
                self._x = []
                self._y = []

    def get(self):
        with self._lock:
            return self._x, self._y

class SeamData():
    
    def __init__(self, size):
        self._lock = Lock()
        self._x = deque()
        self._y = deque()
        self._f = None
        self._h = Header()
        self._size = size

    def from_msg(self, msg: PointCloud2):
        with self._lock:
            if len(msg.data):
                d = rnp.numpify(msg)
                self._x.appendleft(d['x'][0])
                self._y.appendleft(d['y'][0])
            else:
                self._x.appendleft(None)
                self._y.appendleft(None)
            while len(self._x) > self._size:
                self._x.pop()
            while len(self._y) > self._size:
                self._y.pop()

            self._cal_fps(msg.header)

    def _cal_fps(self, h: Header):
        try:
            dt_sec = h.stamp.sec - self._h.stamp.sec
            dt_nano = h.stamp.nanosec - self._h.stamp.nanosec
            dt = dt_sec + dt_nano * 1e-9
            df = int(h.frame_id) - int(self._h.frame_id)
            self._f = df / dt
        except:
            self._f = None
        self._h = h

    def get(self):
        with self._lock:
            x = [x for x in self._x if x is not None]
            y = [y for y in self._y if y is not None]
            return x, y, self._h.frame_id, self._f