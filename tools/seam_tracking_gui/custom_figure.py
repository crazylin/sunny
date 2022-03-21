from matplotlib.figure import Figure
from point_data import PointData

class CustomFigure(Figure):
    """A figure with a text watermark."""

    def __init__(self):
        super().__init__()

        ax = super().add_subplot()
        ax.set_xlabel("X axis (mm)")
        ax.set_ylabel("Y axis (mm)")
        ax.set_title('Graph')
        ax.set_xlim(-10, 80)
        ax.set_ylim(-10, 210)

        self._pnts, = ax.plot([], [], 'b.', label='Laser')
        self._seam, = ax.plot([], [], 'go', label='Seam')
        self._pick, = ax.plot([], [], 'rs', label='Picked')
        self._info = ax.text(10, 190, 'frames:\nfps:')
        self._xxyy = ax.text(40, 190, 'X:\nY:')
        ax.legend()

    # def update_pnts(self, pnts: PointData):
    #     u, v, id, fps = pnts.get()
    #     self._pnts.set_data(u, v)
    #     if fps is None:
    #         self._info.set_text(f'frames: {id:>9}\nfps:')
    #     else:
    #         self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')

    def update_seam(self, seam: PointData):
        x, y, z, id, fps = seam.get()
        pnts_u, pnts_v = [], []
        seam_u, seam_v = [], []
        pick_u, pick_v = [], []
        for a, b, c in zip(x, y, z):
            if z == 0.:
                pnts_u.append(a)
                pnts_v.append(b)
            elif z == 1.:
                seam_u.append(a)
                seam_v.append(b)
            elif z == 2.:
                pick_u.append(a)
                pick_v.append(b)
        self._pnts.set_data(pnts_u, pnts_v)
        self._seam.set_data(seam_u, seam_v)
        self._pick.set_data(pick_u, pick_v)
        if fps is None:
            self._info.set_text(f'frames: {id:>9}\nfps:')
        else:
            self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')
        if len(pick_u):
            self._xxyy.set_text(f"X: {pick_u[0]:>8.2f}\nY: {pick_v[0]:>8.2f}")
        else:
            self._xxyy.set_text(f"X:\nY:")