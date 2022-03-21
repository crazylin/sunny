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
    def __init__(self, s: str = '[]', pos: int = None):
        self._pos = pos
        self[:] = json.loads(s)

    def code(self):
        if self._pos is None:
            return ''
        else:
            return self[self._pos]

    def dump(self, file: str):
        with open(file, 'w') as fp:
            json.dump(self, fp)

    def load(self, file: str):
        with open(file, 'r') as fp:
            self[:] = json.load(fp)
        if len(self):
            if self._pos is None:
                self._pos = 0
            elif self._pos > len(self) - 1:
                self._pos = len(self) - 1
        else:
            self._pos = None

    def loads(self, s: str):
        self[:] = json.loads(s)

    def dumps(self):
        return json.dumps(self)

    def select(self, pos: int):
        self._pos = pos

    def append_code(self, s: str):
        self.append(s)
        self._pos = len(self) - 1

    def delete_code(self):
        del self[self._pos]
        if not len(self):
            self._pos = None
        elif self._pos > len(self) - 1:
            self._pos = len(self) - 1

    def modify_code(self, s: str):
        self[self._pos] = s

    def modify_self(self, s: str):
        self[:] = json.loads(s)
        if len(self):
            if self._pos is None:
                self._pos = 0
            elif self._pos > len(self) - 1:
                self._pos = len(self) - 1
        else:
            self._pos = None

    def previous(self):
        self._pos -= 1

    def next(self):
        self._pos += 1

    def is_begin(self):
        return True if self._pos is None or self._pos == 0 else False

    def is_end(self):
        return True if self._pos is None or self._pos == len(self) - 1 else False

    def is_valid(self):
        return True if self._pos is not None and self._pos <= len(self) - 1 else False

    def pos(self):
        return self._pos