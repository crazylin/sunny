import json
import os

from threading import Lock
from importlib import import_module, reload


class Codes(list):
    lock = Lock()
    dirname = os.path.dirname(__file__)
    modname = 'seam_tracking_code'
    jsonpath = os.path.join(dirname, 'codes.json')
    codepath = os.path.join(dirname, modname + '.py')

    def __init__(self):
        self.load()
        with open(self.codepath, 'w') as f:
            pass
        from . import seam_tracking_code
        self.module = seam_tracking_code

    def __del__(self):
        try:
            os.remove(self.codepath)
        except FileNotFoundError:
            pass

    def __call__(self, *args, **kwargs):
        with self.lock:
            fn = self.module.fn
        return fn(*args, **kwargs)

    def load(self):
        with self.lock, open(self.jsonpath, 'r') as f:
            self[:] = json.load(f)

    def dump(self):
        with self.lock, open(self.jsonpath, 'w') as f:
            json.dump(self, f)

    def reload(self, index):
        with self.lock, open(self.codepath, 'w') as f:
            f.write(self[index])
        reload(self.module)
