from threading import Lock
from shared_interfaces.msg import ModbusCoord

class PickData():
    """For pick data."""

    def __init__(self):
        self._lock = Lock()
        self._u = []
        self._v = []

    def from_msg(self, msg: ModbusCoord):
        if msg.valid:
            self._u = [msg.y]
            self._v = [msg.z]
        else:
            self._u = []
            self._v = []

    def get(self):
        return self._u, self._v