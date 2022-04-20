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

dtype = [('x', np.float32), ('y', np.float32), ('i', np.float32)]


def interpolate(d: np.array):
    if not d:
        return np.array([], dtype=dtype)
    t = []
    for i in range(len(d) - 1):
        t.append((d['x'][i], d['y'][i], d['i'][i]))
        steps = int(round(d['i'][i + 1] - d['i'][i]))
        if steps > 1:
            for s in range(1, steps):
                t.append((None, None, None))
    t.append((d['x'][-1], d['y'][-1], d['i'][-1]))
    return np.array(t, dtype=dtype)


def mid(d: np.array, k: int, t: float):
    z = []
    for i in range(k, len(d) - k):
        y = d['y'][i]
        s = np.sort(d['y'][i - k:i + k + 1])
        if np.isnan(s[k]) or abs(s[k] - y) > t:
            z.append((None, None))
        else:
            z.append((d['x'][i], s[k]))
    return np.array(z, dtype=dtype)


def localMax(d: np.array, *, delta: int):
    md = []
    id = []
    for i in range(delta, len(d) - delta, delta):
        mask = np.invert(np.isnan(d['y'][i:i + delta]))
        if np.any(mask):
            id.append(np.nanargmax(d['y'][i:i + delta]) + i)
    for i in id:
        m = np.nanargmax(d['y'][i - delta + 1:i + delta]) + i - delta + 1
        if i == m:
            md.append(m)
    return md


def localMin(d: np.array, *, delta: int):
    md = []
    id = []
    for i in range(delta, len(d) - delta, delta):
        mask = np.invert(np.isnan(d['y'][i:i + delta]))
        if np.any(mask):
            id.append(np.nanargmin(d['y'][i:i + delta]) + i)
    for i in id:
        m = np.nanargmin(d['y'][i - delta + 1:i + delta]) + i - delta + 1
        if i == m:
            md.append(m)
    return md


def cross(d: np.array, id, delta, num):
    mask1 = np.invert(np.isnan(d['x'][id - delta + 1:id - num]))
    mask2 = np.invert(np.isnan(d['x'][id + num + 1:id + delta]))
    dx1 = d['x'][id - delta + 1:id - num][mask1]
    dy1 = d['y'][id - delta + 1:id - num][mask1]
    dx2 = d['x'][id + num + 1:id + delta][mask2]
    dy2 = d['y'][id + num + 1:id + delta][mask2]
    if np.any(mask1) and np.any(mask2):
        b1, m1 = polyfit(dx1, dy1, 1)
        b2, m2 = polyfit(dx2, dy2, 1)
        px = (b2 - b1) / (m1 - m2)
        py = px * m1 + b1
        return np.array([(px, py)], dtype=dtype)
    else:
        return np.array([], dtype=dtype)


def fn(d: np.array):
    d = interpolate(d)
    d = mid(d, 10, 5.)
    md = localMax(d, delta=150)
    if md:
        return cross(d, md[0], delta=150, num=30)
    else:
        return None


# def fn(d: np.array):
#     if not d:
#         return np.array([], dtype=dtype)
#     id = np.nanargmax(d['y'])
#     if 150 < id < len(d) - 150:
#         return cross(d, id, 150, 30)
#     else:
#         return np.array([], dtype=dtype)
