from functools import wraps
from threading import Lock

def _lock(f):
    @wraps(f)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return f(self, *args, **kwargs)
    return wrapper

class Codes(list):
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Lock()
    
    @_lock
    def __getitem__(self, *args, **kwargs):
        print('get')
        return super().__getitem__(*args, **kwargs)

    @_lock
    def __setitem__(self, *args, **kwargs):
        print('set')
        super().__setitem__(*args, **kwargs)

    @_lock
    def __delitem__(self, *args, **kwargs):
        super().__delitem__(*args, **kwargs)
    
    @_lock
    def __len__(self, *args, **kwargs):
        return super().__len__(*args, **kwargs)

    @_lock
    def insert(self, *args, **kwargs):
        super().insert(*args, **kwargs)