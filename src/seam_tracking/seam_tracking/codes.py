import json
from threading import Lock

class Code():

    def __init__(self):
        self._scope = {}

    def reload(self, s: str):
        self._scope = {}
        exec(s, self._scope)

    def __call__(self, *args, **kwargs):
        return self._scope['fn'](*args, **kwargs)

class Codes(list):

    def __init__(self, file: str):
        self._lock = Lock()
        self._code = Code()
        self._file = file
        self._pos = None

    def _sync(self):
        if len(self):
            if self._pos is None:
                self._pos = 0
            elif self._pos >= len(self):
                self._pos = len(self) - 1
        else:
            self._pos = None

    def __call__(self, *args, **kwargs):
        with self._lock:
            return self._code(*args, **kwargs)

    def load(self):
        with self._lock:
            with open(self._file) as fp:
                self[:] = json.load(fp)
            self._sync()

    def loads(self, s: str):
        with self._lock:
            self[:] = json.loads(s)
            self._sync()

    def dump(self):
        with self._lock, open(self._file) as fp:
            json.dump(self, fp)

    def dumps(self):
        with self._lock:
            return json.dumps(self)

    def reload(self, *, id: int = None):
        with self._lock:
            if id is None:
                self._code.reload(self[self._pos])
            else:
                self._code.reload(self[id])
                self._pos = id

    def get_code(self, *, id: int = None):
        with self._lock:
            if id is None:
                return self[self._pos]
            else:
                return self[id]

    def set_code(self, s: str, *, id: int = None):
        with self._lock:
            if id is None:
                self[self._pos] = s
            else:
                self[id] = s

    def get_codes(self):
        with self._lock:
            return json.dumps(self)

    def set_codes(self, s: str):
        with self._lock:
            self[:] = json.loads(s)
            self._sync()
            with open(self._file, 'w') as fp:
                fp.write(s)
            self._code.reload(self[self._pos])

    def pos(self):
        with self._lock:
            return self._pos
