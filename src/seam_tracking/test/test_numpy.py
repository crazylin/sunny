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
from numpy.polynomial.polynomial import polyfit

dtype = [(x, np.float32) for x in 'xyi']


def interpolate(d: np.array):
    if d.size == 0:
        return np.array([], dtype=dtype)
    length = int(np.max(d['i']))
    ret = np.full((length + 1,), np.nan, dtype=dtype)
    ret[d['i'].astype(np.int64)] = d
    return ret


def local_max(d: np.array, *, delta: int):
    md = []
    id = []
    for i in range(0, d.size, delta):
        mask = ~np.isnan(d['y'][i:i + delta])
        if np.any(mask):
            id.append(np.nanargmax(d['y'][i:i + delta]) + i)
    for i in id:
        if i - delta + 1 < 0 or i + delta > d.size:
            continue
        m = np.nanargmax(d['y'][i - delta + 1:i + delta]) + i - delta + 1
        if i == m:
            md.append(m)
    return md


def local_min(d: np.array, *, delta: int):
    md = []
    id = []
    for i in range(0, d.size, delta):
        mask = ~np.isnan(d['y'][i:i + delta])
        if np.any(mask):
            id.append(np.nanargmin(d['y'][i:i + delta]) + i)
    for i in id:
        if i - delta + 1 < 0 or i + delta > d.size:
            continue
        m = np.nanargmin(d['y'][i - delta + 1:i + delta]) + i - delta + 1
        if i == m:
            md.append(m)
    return md


def cross(d: np.array, id: int, *, delta: int, num: int = None):
    s1 = None if num is None or id - num + 1 < 0 else id - num + 1
    e1 = 0 if id - delta < 0 else id - delta
    s2 = id + delta + 1
    e2 = None if num is None else id + num

    d['i'][s1:e1] = -2
    d['i'][s2:e2] = -3
    mask1 = ~np.isnan(d['x'][s1:e1])
    mask2 = ~np.isnan(d['x'][s2:e2])
    if np.count_nonzero(mask1) > 1 and np.count_nonzero(mask2) > 1:
        b1, m1 = polyfit(d['x'][s1:e1][mask1], d['y'][s1:e1][mask1], 1)
        b2, m2 = polyfit(d['x'][s2:e2][mask2], d['y'][s2:e2][mask2], 1)
        px = (b2 - b1) / (m1 - m2)
        py = px * m1 + b1
        return np.array([
            (px, py, -1), (0, b1, -4),
            (100, 100 * m1 + b1, -4),
            (0, b2, -5),
            (100, 100 * m2 + b2, -5)],
            dtype=dtype)
    else:
        return np.array([], dtype=dtype)


def test_interpolate():
    d = np.array([], dtype=dtype)
    ret = interpolate(d)
    assert ret.size == 0

    d = np.array([(1, 2, 0), (1, 2, 9)], dtype=dtype)
    ret = interpolate(d)
    assert ret.size == 10
    assert ret[0] == np.array([(1, 2, 0)], dtype=dtype)
    assert ret[9] == np.array([(1, 2, 9)], dtype=dtype)


def test_local_max():
    d = np.array([], dtype=dtype)
    pos = local_max(d, delta=3)
    assert len(pos) == 0

    p = [(0., 0., 0.) for i in range(10)]

    d = np.array(p, dtype=dtype)
    d['y'][0] = 10
    pos = local_max(d, delta=3)
    assert len(pos) == 0

    d = np.array(p, dtype=dtype)
    d['y'][1] = 10
    pos = local_max(d, delta=3)
    assert len(pos) == 0

    for i in range(2, 8):
        d = np.array(p, dtype=dtype)
        d['y'][i] = 10
        pos = local_max(d, delta=3)
        assert len(pos) == 1
        assert pos[0] == i

    d = np.array(p, dtype=dtype)
    d['y'][8] = 10
    pos = local_max(d, delta=3)
    assert len(pos) == 0

    d = np.array(p, dtype=dtype)
    d['y'][9] = 10
    pos = local_max(d, delta=3)
    assert len(pos) == 0

    d = np.array(p, dtype=dtype)
    d['y'][2] = 10
    d['y'][7] = 10
    pos = local_max(d, delta=3)
    assert pos[0] == 2
    assert pos[1] == 7

    d = np.array(p, dtype=dtype)
    d['y'][5] = 10
    d['y'][7] = 10
    pos = local_max(d, delta=3)
    assert len(pos) == 1
    assert pos[0] == 5


def test_cross():
    d = np.array([], dtype=dtype)
    cross(d, 10, delta=5)

    l1 = [(i, i, 0) for i in range(10)]
    l2 = [(10 - i, i, 0) for i in range(10, -1, -1)]

    d = np.array(l1 + l2, dtype=dtype)
    # test delta
    cross(d, 10, delta=5)
    for i in range(21):
        if i < 5:
            assert d[i][2] == -2
        elif i > 15:
            assert d[i][2] == -3
        else:
            assert d[i][2] == 0

    d = np.array(l1 + l2, dtype=dtype)
    # test delta = 0
    ret = cross(d, 10, delta=0)
    assert round(ret[0][0]) == 5 and round(ret[0][1]) == 5
    for i in range(21):
        if i < 10:
            assert d[i][2] == -2
        elif i == 10:
            assert d[i][2] == 0
        else:
            assert d[i][2] == -3

    d = np.array(l1 + l2, dtype=dtype)
    # test num
    ret = cross(d, 10, delta=5, num=8)
    assert round(ret[0][0]) == 5 and round(ret[0][1]) == 5
    for i in range(21):
        if i < 3:
            assert d[i][2] == 0
        elif i >= 3 and i < 5:
            assert d[i][2] == -2
        elif i >= 5 and i < 16:
            assert d[i][2] == 0
        elif i >= 16 and i < 18:
            assert d[i][2] == -3
        else:
            assert d[i][2] == 0

    d = np.array(l1 + l2, dtype=dtype)
    # test num when only one point is selected
    ret = cross(d, 10, delta=5, num=7)
    assert ret.size == 0
    for i in range(21):
        if i < 4:
            assert d[i][2] == 0
        elif i == 4:
            assert d[i][2] == -2
        elif i > 4 and i < 16:
            assert d[i][2] == 0
        elif i == 16:
            assert d[i][2] == -3
        else:
            assert d[i][2] == 0

    d = np.array(l1 + l2, dtype=dtype)
    # test num < delta
    cross(d, 10, delta=5, num=3)
    for i in range(21):
        assert d[i][2] == 0
