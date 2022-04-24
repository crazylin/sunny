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

dtype = [('x', np.float32), ('y', np.float32), ('i', np.float32)]


def interpolate(d: np.array):
    if d.size == 0:
        return np.array([], dtype=dtype)
    t = []
    for i in range(len(d) - 1):
        t.append((d['x'][i], d['y'][i], d['i'][i]))
        steps = int(round(d['i'][i + 1] - d['i'][i]))
        if steps > 1:
            for s in range(1, steps):
                t.append((None, None, i + s))
    t.append((d['x'][-1], d['y'][-1], d['i'][-1]))
    return np.array(t, dtype=dtype)


def local_max(d: np.array, *, delta: int):
    md = []
    id = []
    for i in range(0, len(d), delta):
        mask = np.invert(np.isnan(d['y'][i:i + delta]))
        if np.any(mask):
            id.append(np.nanargmax(d['y'][i:i + delta]) + i)
    for i in id:
        if i - delta + 1 < 0 or i + delta > d.size:
            continue
        m = np.nanargmax(d['y'][i - delta + 1:i + delta]) + i - delta + 1
        if i == m:
            md.append(m)
    return md


def test_interpolate():
    d = np.array([], dtype=dtype)
    ret = interpolate(d)
    assert len(ret) == 0

    d = np.array([(1, 2, 0), (1, 2, 10)], dtype=dtype)
    ret = interpolate(d)
    assert len(ret) == 11
    for i in range(11):
        assert ret[i][2] == i


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

def cross_all(d: np.array, id, num):
    for i in range(len(d)):
        if i < id - num:
            d[i][2] = -2
        elif i > id + num:
            d[i][2] = -3
    mask1 = np.invert(np.isnan(d['x'][:id - num]))
    mask2 = np.invert(np.isnan(d['x'][id + num:]))
    if np.any(mask1) and np.any(mask2):
        b1, m1 = polyfit(d['x'][:id - num][mask1], d['y'][:id - num][mask1], 1)
        b2, m2 = polyfit(d['x'][id + num:][mask2], d['y'][id + num:][mask2], 1)
        px = (b2 - b1) / (m1 - m2)
        py = px * m1 + b1
        return np.array([(px, py, -1), (0, b1, -4), (100, 100 * m1 + b1, -4), (0, b2, -5), (100, 100 * m2 + b2, -5)], dtype=dtype)
    else:
        return np.array([], dtype=dtype)


def cross(d: np.array, id, delta, num):
    d1 = d[id - delta + 1:id - num]
    d2 = d[id + num + 1:id + delta]
    for i in range(len(d1)):
        d1[i][2] = -2
    for i in range(len(d2)):
        d2[i][2] = -3
    mask1 = np.invert(np.isnan(d['x'][id - delta + 1:id - num]))
    mask2 = np.invert(np.isnan(d['x'][id + num + 1:id + delta]))
    if np.any(mask1) and np.any(mask2):
        b1, m1 = polyfit(d['x'][id - delta + 1:id - num][mask1], d['y'][id - delta + 1:id - num][mask1], 1)
        b2, m2 = polyfit(d['x'][id + num + 1:id + delta][mask2], d['y'][id + num + 1:id + delta][mask2], 1)
        px = (b2 - b1) / (m1 - m2)
        py = px * m1 + b1
        return np.array([(px, py, -1), (0, b1, -4), (100, 100 * m1 + b1, -4), (0, b2, -5), (100, 100 * m2 + b2, -5)], dtype=dtype)
    else:
        return np.array([], dtype=dtype)