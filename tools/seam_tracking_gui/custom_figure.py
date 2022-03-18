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

    def update_pnts(self, pnts: PointData):
        u, v, id, fps = pnts.get()
        self._pnts.set_data(u, v)
        if fps is None:
            self._info.set_text(f'frames: {id:>9}\nfps:')
        else:
            self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')

    def update_seam(self, seam: PointData):
        u, v, id, fps = seam.get()
        self._pick.set_data(u[:1], v[:1])
        self._seam.set_data(u[1:], v[1:])
        if u and v:
            self._xxyy.set_text(f"X: {u[0]:>8.2f}\nY: {v[0]:>8.2f}")
        else:
            self._xxyy.set_text(f"X:\nY:")