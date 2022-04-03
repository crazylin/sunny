from matplotlib.figure import Figure
from point_data import PointData

class CustomFigure(Figure):
    """A figure with a text watermark."""

    def __init__(self):
        super().__init__()

        self.pnts_data = PointData()
        self.seam_data = PointData()

        bx = [-20, 33, 110, 152, -20]
        by = [400, -12, -12, 400, 400]

        ax = super().add_subplot()
        ax.set_xlabel("X axis (mm)")
        ax.set_ylabel("Y axis (mm)")
        ax.set_title('Graph')
        ax.set_xlim(-30, 160)
        ax.set_ylim(-20, 420)

        self._pnts, = ax.plot([], [], 'b.', label='Laser')
        self._seam, = ax.plot([], [], 'go', label='Seam')
        self._pick, = ax.plot([], [], 'rs', label='Picked')
        ax.plot(bx, by, linewidth=3)
        self._info = ax.text(0, 370, 'frames:\nfps:')
        self._xxyy = ax.text(60, 370, 'X:\nY:')
        ax.legend()

    def update_seam(self, *args):
        x, y, id, fps = self.seam_data.get()
        self._pick.set_data(x[:1], y[:1])
        self._seam.set_data(x[1:], y[1:])
        if fps is None:
            self._info.set_text(f'frames: {id:>9}\nfps:')
        else:
            self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')
        if len(x) and len(y):
            self._xxyy.set_text(f"X: {x[0]:>8.2f}\nY: {y[0]:>8.2f}")
        else:
            self._xxyy.set_text(f"X:\nY:")

    def update_pnts(self, *args):
        x, y, *t = self.pnts_data.get()
        self._pnts.set_data(x[:], y[:])

    def msg_to_seam(self, msg):
        self.seam_data.from_msg(msg)

    def msg_to_pnts(self, msg):
        self.pnts_data.from_msg(msg)