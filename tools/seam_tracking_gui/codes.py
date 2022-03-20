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
        self.append()
        self._str = ''
        self._pos = None


