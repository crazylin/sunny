import json

class Codes(list):

    _info = '''\
def fn(x: list, y: list, u: list, v: list):
    """
    Callback function to locate seam.

    Extended.

    Parameters
    ----------
    x : list
        Object domain x
    y : list
        Object domain y
    u : list
        Image domain x, column
    v : list
        Image domain y, row

    Returns
    -------
    Tuple[list, list]
        A pair of list of coordinates
    """
    
    # To be filled
    return [], []
'''
    def __init__(self):
        self._pos = None

    def _sync(self):
        if len(self):
            if self._pos is None:
                self._pos = 0
            elif self._pos >= len(self):
                self._pos = len(self) - 1
        else:
            self._pos = None

    def code(self, *, id: int = None):
        if id is None:
            if self._pos is None:
                return ''
            else:
                return self[self._pos]
        else:
            return self[id]

    def dump(self, file: str):
        with open(file, 'w') as fp:
            json.dump(self, fp)

    def load(self, file: str):
        with open(file, 'r') as fp:
            self[:] = json.load(fp)
        self._sync()

    def loads(self, s: str):
        self[:] = json.loads(s)
        self._sync()

    def dumps(self):
        return json.dumps(self)

    def append_code(self, s: str):
        self.append(s)
        self._pos = len(self) - 1

    def delete_code(self):
        del self[self._pos]
        self._sync()

    def modify_code(self, s: str):
        self[self._pos] = s

    def previous(self):
        if self._pos > 0:
            self._pos -= 1

    def next(self):
        if self._pos < len(self) - 1:
            self._pos += 1

    def goto(self, *, id: int):
        if id is not None and 0 < id < len(self):
            self._pos = id

    def is_begin(self):
        return True if self._pos is None or self._pos == 0 else False

    def is_end(self):
        return True if self._pos is None or self._pos == len(self) - 1 else False

    def is_valid(self):
        return True if self._pos is not None and self._pos <= len(self) - 1 else False

    def pos(self):
        return self._pos
