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

import perspect


def test_perspect():
    src = [(0, 0), (1, 0), (1, 1), (0, 1)]
    dst = [(0, 80), (100, 80), (100, 0), (0, 0)]
    H = perspect.findHomography(src, dst)
    src.append((0.5, 0.5))
    dst.append((50, 40))
    newsrc = perspect.perspectiveTransform(src, H)
    newdst = perspect.perspectiveTransform(dst, H, inv=True)
    assert newsrc == dst
    assert newdst == src

    src = [(0, 0), (1, 0), (1, 1), (0, 1)]
    dstA = [(0, 80), (100, 80), (100, 0), (0, 0)]
    dstB = [(0, 50), (50, 50), (50, 0), (0, 0)]
    H = perspect.findHomography(src, dstA)
    Z = perspect.getNewHomography(dstA, dstB, H, remap=(0, 2, 0, 2))
    B = perspect.perspectiveTransform([(0, 0), (2, 2)], Z)
    assert B == [(-50.0, 100.0), (50.0, 0.0)]

    # m = [
    #     8.95810953e-02,  1.09862135e-01, -1.93995953e+01,
    #     -4.15779030e-01,  4.70428332e-03,  3.99283020e+02,
    #     1.16829295e-03, -1.26498465e-05,  1.00000000e+00]

    # ori = [(0, 0), (1024, 0), (1024, 1536), (0, 1536)]
    src = [
        (24.124022, 122.92669),
        (47.70333, 18.13594),
        (86.729355, 18.226416),
        (114.363655, 125.04152)]
    # m = [
    #     8.95810953e-02,  1.09862135e-01, -1.93995953e+01,
    #     -4.15779030e-01,  4.70428332e-03,  3.99283020e+02,
    #     1.16829295e-03, -1.26498465e-05,  1.00000000e+00]
    # dst = [(0, 80), (27.5, 0), (57.5, 0), (85, 80)]
    # ori = [(0, 0), (1024, 0), (1024, 1536), (0, 1536)]
