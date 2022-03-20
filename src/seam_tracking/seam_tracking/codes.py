import json
import os
import importlib
from threading import Lock


class Codes(list):
    _lock = Lock()
    _dirname = os.path.dirname(__file__)
    _modname = 'seam_tracking_code'
    _jsonpath = os.path.join(_dirname, 'codes.json')
    _codepath = os.path.join(_dirname, _modname + '.py')
    with open(_codepath, 'w') as f:
        pass
    from . import seam_tracking_code as stc

    def __init__(self, *, id = 0):
        self._id = 0
        self.append('def fn(x: list, y: list):\n    return [], []')
        self.load()
        self.reload(id=id)

    def __del__(self):
        try:
            os.remove(self._codepath)
        except FileNotFoundError:
            pass

    def __call__(self, *args, **kwargs):
        with self._lock:
            return self.stc.fn(*args, **kwargs)

    def load(self):
        with self._lock, open(self._jsonpath, 'r') as f:
            self[1:] = json.load(f)

    def loads(self, s):
        self[:] = json.loads(s)

    def dump(self):
        with self._lock, open(self._jsonpath, 'w') as f:
            json.dump(self[1:], f)

    def dumps(self):
        with self._lock:
            return json.dumps(self[1:]), self._id

    def reload(self, *, id = None):
        with self._lock:
            if id is None:
                id = self._id if self._id < len(self) else len(self) - 1
            with open(self._codepath, 'w') as f:
                f.write(self[id])
            importlib.reload(self.stc)
            self._id = id

    def get_code(self, *, id = None):
        with self._lock:
            id = self._id if id is None else id
            return self[id]

    def set_code(self, code, *, id = None):
        with self._lock:
            id = self._id if id is None else id
            self[id] = code

    def get_codes(self):
        with self._lock:
            return json.dumps(self), self._id

    # def set_codes(self, s):
    #     with self._lock:
    #         self._loads(s)

    def get_id(self):
        with self._lock:
            return self._id
