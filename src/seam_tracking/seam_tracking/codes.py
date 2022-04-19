"""Provide a Codes as list of string.
"""

# import json
from functools import wraps
from threading import RLock

def _lock(f):
    """Function decorater to protect from data race.

    Args:
        f (_type_): Callable object.

    Returns:
        _type_: Wrapped callable object.
    """
    @wraps(f)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return f(self, *args, **kwargs)
    return wrapper

class _Code():
    """A local namespace wraps names in source code.

    Source code in string form will be executed.
    Names are saved in local scope from which the fn shall find.
    """

    def __init__(self):
        self._scope = {}

    def reload(self, s: str):
        self._scope = {}
        exec(s, self._scope)

    def __call__(self, *args, **kwargs):
        return self._scope['fn'](*args, **kwargs)

class Codes(list):
    """A list of strings protected from data race.

    Args:
        list (_type_): Inherit from build-in list.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = RLock()
        self._code = _Code()

    @_lock
    def __getitem__(self, *args, **kwargs):
        return super().__getitem__(*args, **kwargs)

    @_lock
    def __setitem__(self, *args, **kwargs):
        return super().__setitem__(*args, **kwargs)

    @_lock
    def __delitem__(self, *args, **kwargs):
        return super().__delitem__(*args, **kwargs)

    @_lock
    def __len__(self, *args, **kwargs):
        return super().__len__(*args, **kwargs)

    @_lock
    def __call__(self, *args, **kwargs):
        return self._code(*args, **kwargs)

    # @_lock
    # def load(self, file: str):
    #     with open(file) as fp:
    #         self[:] = json.load(fp)

    # @_lock
    # def loads(self, s: str):
    #     self[:] = json.loads(s)

    # @_lock
    # def dump(self, file: str):
    #     with open(file) as fp:
    #         json.dump(self, fp)

    # @_lock
    # def dumps(self):
    #     return json.dumps(self)

    @_lock
    def reload(self, id: int):
        self._code.reload(self[id])
