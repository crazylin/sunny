import numpy as np
import ros2_numpy as rnp
from threading import Lock
from numpy.polynomial.polynomial import polyfit


def Calculate(x, y, num, delta):
    index = y.index(max(y))
    if index - num < 0 or index + num > len(y):
        return None, None
    b1, m1 = polyfit(x[index - num : index - delta], y[index - num : index - delta], 1)
    b2, m2 = polyfit(x[index + delta : index + num], y[index + delta : index + num], 1)
    try:
        px = (b1 - b2) / (m2 - m1)
        py = px * m1 + b1
    except:
        return None, None
    else:
        return px, py
    
def Convolve(x, y, k = 10, t = 5):
    dy = [0.] * len(y)
    for i in range(k, len(y) - k):
        for j in range(-k, k + 1):
            dy[i] += y[i] - y[i + j]
    v = min(dy)
    return dy.index(v) if v < -t else None

def segment(line, dx, dy, count = 1):
    segX, segY = [], []
    # if not x or not y:
    #     return segX, segY
    
    L = list(zip(line[0], line[1]))

    while(L):
        (X, Y), *L = L
        tmpX, tmpY = [X], [Y]
        while(L):
            if abs(X - L[0][0]) > dx or abs(Y - L[0][1]) > dy:
                break
            else:
                (X, Y), *L = L
                tmpX.append(X)
                tmpY.append(Y)
        if len(tmpX) > count:
            segX.append(tmpX)
            segY.append(tmpY)

    return [segX, segY]

def candidates(segs):
    cx, cy = [], []
    for lx, ly in zip(segs[0], segs[1]):
        cx.extend([lx[0], lx[-1]])
        cy.extend([ly[0], ly[-1]])

    return [cx, cy]

def pick(pnts, index):
    try:
        return [[pnts[0][index]], [pnts[1][index]]]
    except Exception:
        return [[], []]

def Exe(line, dx, dy, count = 1):
    segs = segment(line, dx, dy, count)
    return candidates(segs)

def msg2dict(msg):
    if msg.height * msg.width == 0:
        return msg.header, [[], []]
    else:
        d = rnp.numpify(msg)
        return msg.header, [d['y'].tolist(), d['z'].tolist()]

class LineData():
    """For line data."""

    def __init__(self):
        self._lock = Lock()
        self._data = [[], []]
        self._sec = 0
        self._nanosec = 0
        self._id = 0
        self._fps = 0

    def write(self, header, data):
        with self._lock:
            self._data = data
            id = int(header.frame_id)
            self._fps = (id - self._id) / (header.stamp.sec - self._sec + (header.stamp.nanosec - self._nanosec) * 1e-9)
            self._sec = header.stamp.sec
            self._nanosec = header.stamp.nanosec
            self._id = id

    def read(self):
        with self._lock:
            return self._data, self._id, self._fps

class PickData():
    """For pick data."""

    def __init__(self):
        self._lock = Lock()
        self._data = [[], []]

    def write(self, data):
        with self._lock:
            self._data = data

    def read(self):
        with self._lock:
            return self._data
