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

from matplotlib.figure import Figure
from point_data import SeamData


class CustomFigure(Figure):
    """A figure with a text watermark."""

    def __init__(self):
        super().__init__()

        self._pd = {
            "laser": {
                "cb": lambda i: i >= 0,
                "x": [],
                "y": [],
                "fmt": ".b",
                "kwargs": {
                    "label": "Laser",
                    "markersize": 1.
                }
            },
            "pick": {
                "cb": lambda i: i == -1,
                "x": [],
                "y": [],
                "fmt": "sr",
                "kwargs": {
                    "label": "Pick",
                    "markersize": 5.
                }
            },
            "pnts_a": {
                "cb": lambda i: i == -2,
                "x": [],
                "y": [],
                "fmt": "om",
                "kwargs": {
                    "label": "Points A",
                    "markersize": 3.
                }
            },
            "pnts_b": {
                "cb": lambda i: i == -3,
                "x": [],
                "y": [],
                "fmt": "oy",
                "kwargs": {
                    "label": "Points B",
                    "markersize": 3.
                }
            },
            "line_a": {
                "cb": lambda i: i == -4,
                "x": [],
                "y": [],
                "fmt": "-m",
                "kwargs": {
                    "label": "Line A",
                    "linewidth": 1.
                }
            },
            "line_b": {
                "cb": lambda i: i == -5,
                "x": [],
                "y": [],
                "fmt": "-y",
                "kwargs": {
                    "label": "Line B",
                    "linewidth": 1.
                }
            }
        }

        self.seam_data = SeamData()

        ax = super().add_subplot()
        ax.set_xlabel("X axis (mm)")
        ax.set_ylabel("Y axis (mm)")
        ax.set_title('Graph')
        ax.set_xlim(-30, 160)
        ax.set_ylim(-20, 420)
        self._info = ax.text(0, 370, 'frames:\nfps:')
        self._xxyy = ax.text(60, 370, 'X:\nY:')
        ax.plot([-20, 33, 110, 152, -20], [400, -12, -12, 400, 400], "--b")
        for v in self._pd.values():
            v['handle'], = ax.plot([], [], v['fmt'], **v['kwargs'])
        ax.legend()

    def update_seam(self, *args):
        x, y, i, id, fps = self.seam_data.get()

        for v in self._pd.values():
            v['x'] = [a for a, b in zip(x, i) if v['cb'](b)]
            v['y'] = [a for a, b in zip(y, i) if v['cb'](b)]
            v["handle"].set_data(v['x'], v['y'])
            x = [a for a, b in zip(x, i) if not v['cb'](b)]
            y = [a for a, b in zip(y, i) if not v['cb'](b)]
            i = [a for a in i if not v['cb'](a)]

        if fps is None:
            self._info.set_text(f'frames: {id:>9}\nfps:')
        else:
            self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')
        if self._pd['pick']['x'] and self._pd['pick']['y']:
            self._xxyy.set_text(
                f"X: {self._pd['pick']['x'][0]:>8.2f}\nY: {self._pd['pick']['y'][0]:>8.2f}")
        else:
            self._xxyy.set_text(f"X:\nY:")

    def msg_to_seam(self, msg):
        self.seam_data.from_msg(msg)
