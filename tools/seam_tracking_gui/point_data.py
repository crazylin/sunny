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

from functools import wraps
from threading import Lock
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import numpy as np
import ros2_numpy as rnp

dtype = [(x, np.float32) for x in 'xyi']


def _lock(f):
    """Decorater to protect from data race."""
    @wraps(f)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return f(self, *args, **kwargs)
    return wrapper


class SeamData():
    """For point(x, y) data."""

    def __init__(self):
        self._lock = Lock()
        self._d = np.array([], dtype=dtype)
        self._pid = 0
        self._fps = None
        self._header = Header()
        self._traj = np.full((1800, ), np.nan, dtype=dtype)

    @_lock
    def export_data(self, filename: str):
        if filename is None:
            return
        with open(filename, 'w') as fp:
            for d in self._d:
                fp.write(f'{d}\n')

    @_lock
    def export_traj(self, filename: str):
        if filename is None:
            return
        with open(filename, 'w') as fp:
            for d in self._traj:
                fp.write(f'{d}\n')

    @_lock
    def from_msg(self, msg: PointCloud2):
        id = int(msg.header.frame_id) % 1800
        if msg.data:
            self._d = rnp.numpify(msg)
            self._traj[id] = self._d[0]
        else:
            self._d = np.array([], dtype=dtype)
            self._traj[id] = (np.nan, np.nan, np.nan)

        if id < self._pid:
            self._traj[self._pid + 1:] = (np.nan, np.nan, np.nan)
            self._traj[:id] = (np.nan, np.nan, np.nan)
        else:
            self._traj[self._pid + 1:id] = (np.nan, np.nan, np.nan)
        self._pid = id
        self._fps = self._cal_fps(msg.header)
        self._header = msg.header

    def _cal_fps(self, h: Header):
        try:
            dt_sec = h.stamp.sec - self._header.stamp.sec
            dt_nano = h.stamp.nanosec - self._header.stamp.nanosec
            dt = dt_sec + dt_nano * 1e-9
            df = int(h.frame_id) - int(self._header.frame_id)
            return df / dt
        except Exception:
            return None

    @_lock
    def get(self):
        return self._d, self._header.frame_id, self._fps

    @_lock
    def get_trajectory(self):
        return self._traj.copy(), self._header.frame_id

# class SeamData():

#     def __init__(self, size):
#         self._lock = Lock()
#         self._x = deque()
#         self._y = deque()
#         self._f = None
#         self._h = Header()
#         self._size = size

#     def from_msg(self, msg: PointCloud2):
#         with self._lock:
#             if len(msg.data):
#                 d = rnp.numpify(msg)
#                 self._x.appendleft(d['x'][0])
#                 self._y.appendleft(d['y'][0])
#             else:
#                 self._x.appendleft(None)
#                 self._y.appendleft(None)
#             while len(self._x) > self._size:
#                 self._x.pop()
#             while len(self._y) > self._size:
#                 self._y.pop()

#             self._cal_fps(msg.header)

#     def _cal_fps(self, h: Header):
#         try:
#             dt_sec = h.stamp.sec - self._h.stamp.sec
#             dt_nano = h.stamp.nanosec - self._h.stamp.nanosec
#             dt = dt_sec + dt_nano * 1e-9
#             df = int(h.frame_id) - int(self._h.frame_id)
#             self._f = df / dt
#         except:
#             self._f = None
#         self._h = h

#     def get(self):
#         with self._lock:
#             x = [x for x in self._x if x is not None]
#             y = [y for y in self._y if y is not None]
#             return x, y, self._h.frame_id, self._f
