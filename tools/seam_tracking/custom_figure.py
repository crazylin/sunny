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

        self.__line, = ax.plot([], [], 'b.', label='Laser')
        # self.__pnts, = ax.plot([], [], 'yo', label='Candidates')
        self.__pick, = ax.plot([], [], 'rs', label='Picked')
        self.__info = ax.text(0, 190, 'frames:\nfps:')
        self.__xxyy = ax.text(40, 190, 'x:\ny:')
        ax.legend()

    def update_line(self, line_data):
        info, line = line_data.read()
        self.__info.set_text(info)
        self.__line.set_data(line[0], line[1])
        # self.__pnts.set_data(pnts[0], pnts[1])
        # self.__pick.set_data(pick[0], pick[1])
    
    def update_pick(self, pick_data):
        pick = pick_data.read()
        if len(pick[0]) != 0:
            self.__xxyy.set_text(f"\nx: {pick[0][0]:.2f}\ny: {pick[1][0]:.2f}")
        self.__pick.set_data(pick[0], pick[1])

    
