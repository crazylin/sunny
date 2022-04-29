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

import numpy as np
from collections import deque

dtype = [(x, np.float32) for x in 'xyi']


class Filter:
    def __init__(self):
        self._deq = deque()
        self._enable = True
        self._ws = 10
        self._gap = 2
        self._step = 2.
        self._length = 5

    def _filter(self, r: np.ndarray):
        if r.size and r[0][2] == -1:
            self._deq.appendleft(r[0][1])
        else:
            self._deq.appendleft(None)

        while len(self._deq) > self._ws:
            self._deq.pop()

        if not self._enable or self._deq[0] is None:
            return r

        i = 0
        j = 1
        while j < len(self._deq):
            if j - i > self._gap or i >= self._length:
                break
            if self._deq[j] is None:
                j += 1
                continue
            d = abs(self._deq[j] - self._deq[i]) / (j - i)
            if d > self._step:
                j += 1
            else:
                i = j
                j += 1

        if i >= self._length:
            return r
        else:
            r[0][2] = -8
            return r


def test_length():
    d = np.array([(0, 0, -1)], dtype=dtype)

    filter = Filter()
    filter._length = 5
    for i in range(5):
        ret = filter._filter(d.copy())
        assert ret[0][2] == -8

    ret = filter._filter(d.copy())
    assert ret[0][2] == -1

    filter = Filter()
    filter._length = 6
    for i in range(6):
        ret = filter._filter(d.copy())
        assert ret[0][2] == -8

    ret = filter._filter(d.copy())
    assert ret[0][2] == -1


def test_enable():
    d = np.array([(0, 0, -1)], dtype=dtype)

    filter = Filter()
    filter._enable = False
    for i in range(5):
        ret = filter._filter(d.copy())
        assert ret[0][2] == -1


def test_gap_step():
    d = np.array([(0, 0, -1)], dtype=dtype)
    z = np.array([(0, 0, -2)], dtype=dtype)
    u = np.array([(0, 4, -1)], dtype=dtype)

    filter = Filter()
    filter._gap = 2
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(z.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    assert ret[0][2] == -1

    filter = Filter()
    filter._gap = 2
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(z.copy())
    ret = filter._filter(z.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    assert ret[0][2] == -8

    filter = Filter()
    filter._gap = 2
    filter._step = 2
    ret = filter._filter(u.copy())
    ret = filter._filter(u.copy())
    ret = filter._filter(z.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    assert ret[0][2] == -8
    ret = filter._filter(d.copy())
    assert ret[0][2] == -1

    filter = Filter()
    filter._gap = 2
    filter._step = 1.999
    ret = filter._filter(u.copy())
    ret = filter._filter(u.copy())
    ret = filter._filter(z.copy())
    ret = filter._filter(d.copy())
    ret = filter._filter(d.copy())
    assert ret[0][2] == -8
    ret = filter._filter(d.copy())
    assert ret[0][2] == -8
