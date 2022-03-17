from matplotlib.figure import Figure
from point_data import PointData

class CustomFigure(Figure):
    """A figure with a text watermark."""

    def __init__(self):
        super().__init__()

        ax = super().add_subplot()
        ax.set_xlabel("Y axis (mm)")
        ax.set_ylabel("Z axis (mm)")
        ax.set_title('Graph')
        ax.set_xlim(-10, 80)
        ax.set_ylim(-10, 210)

        self._line, = ax.plot([], [], 'b.', label='Laser')
        self._pick, = ax.plot([], [], 'rs', label='Picked')
        self._info = ax.text(0, 190, 'frames:\nfps:')
        self._xxyy = ax.text(40, 190, 'y:\nz:')
        ax.legend()

    def update_line(self, line: PointData):
        u, v, id, fps = line.get()
        self._line.set_data(u, v)
        if fps == None:
            self._info.set_text(f'frames: {id:>9}\nfps:')
        else:
            self._info.set_text(f'frames: {id:>9}\nfps: {fps:>16.2f}')

    def update_pick(self, pick: PointData):
        u, v, *t = pick.get()
        self._pick.set_data(u, v)
        if u and v:
            self._xxyy.set_text(f"y: {u[0]:>8.2f}\nz: {v[0]:>8.2f}")
        else:
            self._xxyy.set_text(f"y:\nz:")

    
