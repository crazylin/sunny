"""Provide a Codes as list of string."""

# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import json
from functools import wraps
from threading import RLock


def _lock(f):
    """Decorater to protect from data race."""
    @wraps(f)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return f(self, *args, **kwargs)
    return wrapper


class _Code():
    """
    A local namespace wraps names in source code.

    Source code in string form will be executed.
    Names are saved in local scope from which the fn shall find.
    """

    def __init__(self):
        """Initialize self with a local namespace."""
        self._scope = {}

    def reload(self, s: str):
        """
        Reload, execute code via exec and saved as a local namespace.

        :param s: Code to execute.
        :type s: str
        """
        self._scope = {}
        exec(s, self._scope)

    def __call__(self, *args, **kwargs):
        """
        Forward calls to fn in local namespace.

        The fn signature receives a np.array with data type as:
        dtype([('x', np.float32), ('y', np.float32), ('i', np.float32)]).
        It is expected to return a np.array with the same data type as the input.
        The downstream nodes GUI for example can utilize extra infomations encoded in 'i' like:
        Laser line points: i >= 0
        Picked seam: i == -1
        Points selected to fit line A: i == -2
        Points selected to fit line B: i == -3
        Line A: i == -4
        Line B: i == -5

        :return: The result of fn.
        :rtype: np.array
        """
        return self._scope['fn'](*args, **kwargs)


class Codes(list):
    """
    Customized code list to protect from data race.

    Behave exactly as build-in list.
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

    @_lock
    def reload(self, id: int):
        self._code.reload(self[id])
