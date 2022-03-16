from matplotlib.figure import Figure

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
        self._xxyy = ax.text(40, 190, 'x:\ny:')
        ax.legend()

    def update_line(self, line_data):
        data, id, fps = line_data.read()
        self._info.set_text(f'frames: {id:8}\nfps: {fps:15.2f}')
        self._line.set_data(data[0], data[1])

    def update_pick(self, pick_data):
        pick = pick_data.read()
        if len(pick[0]) != 0:
            self._xxyy.set_text(f"x: {pick[0][0]:.2f}\ny: {pick[1][0]:.2f}")
        self._pick.set_data(pick[0], pick[1])

    
