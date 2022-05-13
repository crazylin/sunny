﻿# Copyright 2019 Zhushi Tech, Inc.
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
from matplotlib.figure import Figure
from point_data import SeamData

seam_data = SeamData()
# bound_x = [0, 118, 175, 260, 0]
# bound_y = [519, 0, 0, 519, 519]
min_x = -100
min_y = 0
max_x = 100
max_y = 300


def msg_to_seam(msg):
    seam_data.from_msg(msg)


def export_data(filename: str):
    seam_data.export_data(filename)


def export_traj(filename: str):
    seam_data.export_traj(filename)


class CustomFigure(Figure):
    """A figure with a text watermark."""

    def __init__(self):
        super().__init__()

        self._pd = {
            "laser": {
                # "cb": lambda i: i >= 0,
                # "x": [],
                # "y": [],
                "fmt": ".b",
                "kwargs": {
                    "label": "Laser",
                    "markersize": 1.
                }
            },
            "pick": {
                # "cb": lambda i: i == -1,
                # "x": [],
                # "y": [],
                "fmt": "sr",
                "kwargs": {
                    "label": "Pick",
                    "markersize": 5.
                }
            },
            "pnts_a": {
                # "cb": lambda i: i == -2,
                # "x": [],
                # "y": [],
                "fmt": "om",
                "kwargs": {
                    "label": "Points A",
                    "markersize": 3.
                }
            },
            "pnts_b": {
                # "cb": lambda i: i == -3,
                # "x": [],
                # "y": [],
                "fmt": "oy",
                "kwargs": {
                    "label": "Points B",
                    "markersize": 3.
                }
            },
            "line_a": {
                # "cb": lambda i: i == -4,
                # "x": [],
                # "y": [],
                "fmt": "-m",
                "kwargs": {
                    "label": "Line A",
                    "linewidth": 1.
                }
            },
            "line_b": {
                # "cb": lambda i: i == -5,
                # "x": [],
                # "y": [],
                "fmt": "-y",
                "kwargs": {
                    "label": "Line B",
                    "markersize": 3.
                }
            },
            "src": {
                "fmt": "xr",
                "kwargs": {
                    "label": "Source",
                }
            },
            "bound_1": {
                "fmt": "--b",
                "kwargs": {
                    "linewidth": 1.
                }
            },
            "bound_2": {
                "fmt": "--b",
                "kwargs": {
                    "linewidth": 1.
                }
            }
        }

        ax = super().add_subplot()
        ax.set_xlabel("X axis (mm)")
        ax.set_ylabel("Y axis (mm)")
        ax.set_title('Graph')
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        self._info = ax.text(0.1, 0.9, 'frames:\nfps:', transform=ax.transAxes)
        self._xxyy = ax.text(0.5, 0.9, 'X:\nY:', transform=ax.transAxes)
        # ax.plot(bound_x, bound_y, "--b")
        for v in self._pd.values():
            v['handle'], = ax.plot([], [], v['fmt'], **v['kwargs'])
        ax.legend(loc='lower left')
        self._ax = ax

    def update_seam(self, *args):
        d, id, fps = seam_data.get()
        if int(id) % 2 == 0:
            return

        mask = d['i'] >= 0
        self._pd['laser']['handle'].set_data(d['x'][mask], d['y'][mask])

        mask = d['i'] == -2
        self._pd['pnts_a']['handle'].set_data(d['x'][mask], d['y'][mask])

        mask = d['i'] == -3
        self._pd['pnts_b']['handle'].set_data(d['x'][mask], d['y'][mask])

        mask = d['i'] == -4
        self._pd['line_a']['handle'].set_data(d['x'][mask], d['y'][mask])

        mask = d['i'] == -5
        self._pd['line_b']['handle'].set_data(d['x'][mask], d['y'][mask])

        if d.size and d[0][2] == -1:
            self._pd['pick']['handle'].set_data(d['x'][:1], d['y'][:1])
            self._xxyy.set_text(
                f"X: {d['x'][0]:>8.2f}\nY: {d['y'][0]:>8.2f}")
        else:
            self._pd['pick']['handle'].set_data([], [])
            self._xxyy.set_text(f"X:\nY:")

        if fps is None:
            self._info.set_text(f'frames: {id:>9}\nfps:')
        else:
            self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')

        self.canvas.draw_idle()

    def update_limit(self, x, y):
        self._ax.set_xlim(x[0], x[1])
        self._ax.set_ylim(y[0], y[1])
        self.canvas.draw_idle()

    def update_src(self, l: list):
        x = [i for (i, j) in l if i is not None]
        y = [j for (i, j) in l if j is not None]
        self._pd['src']['handle'].set_data(x, y)
        self.canvas.draw_idle()

    def update_bound(self, a, b, c, d):
        self._pd['bound_1']['handle'].set_data(a, b)
        self._pd['bound_2']['handle'].set_data(c, d)
        self.canvas.draw_idle()


class CustomFigureT(Figure):
    """A figure with a text watermark."""

    def __init__(self):
        super().__init__()

        ax = super().add_subplot(211)
        ax.set_ylabel("X axis (mm)")
        ax.set_title('Trajectory')
        ax.set_xlim(0, 1800)
        ax.set_ylim(min_x, max_x)
        self.plot_x_pick, = ax.plot([], [], '.b', label='pushed', markersize=3)
        self.plot_x_move, = ax.plot([], [], 'sr', label='moved', markersize=3)
        ax.legend(loc='upper right')

        ay = super().add_subplot(212, sharex=ax)
        # ay.set_xlabel("Frame ID")
        ay.set_ylabel("Y axis (mm)")
        # ay.set_xlim(0, 1800)
        ay.set_ylim(min_y, max_y)
        self.plot_y_pick, = ay.plot([], [], '.b', label='pushed', markersize=3)
        self.plot_y_move, = ay.plot([], [], 'sr', label='moved', markersize=3)
        ay.legend(loc='upper right')

        self._ax = ax
        self._ay = ay

    def update_seam(self, *args):
        d, id = seam_data.get_trajectory()
        if int(id) % 2 == 0:
            return

        mask, = np.nonzero(d['i'] == -1)
        if mask.size:
            self.plot_x_pick.set_data(mask, d['x'][mask])
            self.plot_y_pick.set_data(mask, d['y'][mask])
        else:
            self.plot_x_pick.set_data([], [])
            self.plot_y_pick.set_data([], [])

        mask, = np.nonzero(d['i'] == -8)
        if mask.size:
            self.plot_x_move.set_data(mask, d['x'][mask])
            self.plot_y_move.set_data(mask, d['y'][mask])
        else:
            self.plot_x_move.set_data([], [])
            self.plot_y_move.set_data([], [])

        self.canvas.draw_idle()

    def update_limit(self, x, y):
        self._ax.set_ylim(x[0], x[1])
        self._ay.set_ylim(y[0], y[1])
        self.canvas.draw_idle()
