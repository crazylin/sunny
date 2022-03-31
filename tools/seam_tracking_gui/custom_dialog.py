import tkinter as tk
from tkinter.simpledialog import Dialog

class MyDialog(Dialog):
    def __init__(self, parent, title, *, initialvalue=('', '')):
        self._x = initialvalue[0]
        self._y = initialvalue[1]
        super().__init__(parent, title)

    def body(self, frame):
        x = tk.Frame(frame)
        self.x_label = tk.Label(x, width=10, text="Offset x:")
        self.x_label.pack(side=tk.LEFT)
        self.x_box = tk.Entry(x, width=15)
        self.x_box.insert(tk.END, str(self._x))
        self.x_box.pack(side=tk.LEFT)
        self.x_unit = tk.Label(x, width=5, text="mm")
        self.x_unit.pack(side=tk.LEFT)
        x.pack()

        y = tk.Frame(frame)
        self.y_label = tk.Label(y, width=10, text="Offset y:")
        self.y_label.pack(side=tk.LEFT)
        self.y_box = tk.Entry(y, width=15)
        self.y_box.insert(tk.END, str(self._y))
        self.y_box.pack(side=tk.LEFT)
        self.y_unit = tk.Label(y, width=5, text="mm")
        self.y_unit.pack(side=tk.LEFT)
        y.pack()

        return frame

    def ok_pressed(self):
        try:
            self._x = float(self.x_box.get())
            self._y = float(self.y_box.get())
        except Exception as e:
            pass
        else:
            self.destroy()

    def cancel_pressed(self):
        self._x = None
        self._y = None
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side="left")
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side="right")
        self.bind("<Return>", lambda event: self.ok_pressed())
        self.bind("<Escape>", lambda event: self.cancel_pressed())

def mydialog(app, *, initialvalue=('', '')):
    dialog = MyDialog(title="Offset", parent=app, initialvalue=initialvalue)
    return dialog._x, dialog._y

class FilterDialog(Dialog):
    def __init__(self, parent, title, *, initialvalue=('', '', '', '', '')):
        self._ws = initialvalue[0]
        self._gap = initialvalue[1]
        self._dev = initialvalue[2]
        self._step = initialvalue[3]
        self._length = initialvalue[4]
        super().__init__(parent, title)

    def body(self, frame):
        ws = tk.Frame(frame)
        self.ws_label = tk.Label(ws, width=10, text="window size:")
        self.ws_label.pack(side=tk.LEFT)
        self.ws_box = tk.Entry(ws, width=15)
        self.ws_box.insert(tk.END, str(self._ws))
        self.ws_box.pack(side=tk.LEFT)
        self.ws_unit = tk.Label(ws, width=5, text="pixel")
        self.ws_unit.pack(side=tk.LEFT)
        ws.pack()

        gap = tk.Frame(frame)
        self.gap_label = tk.Label(gap, width=10, text="gap:")
        self.gap_label.pack(side=tk.LEFT)
        self.gap_box = tk.Entry(gap, width=15)
        self.gap_box.insert(tk.END, str(self._gap))
        self.gap_box.pack(side=tk.LEFT)
        self.gap_unit = tk.Label(gap, width=5, text="pixel")
        self.gap_unit.pack(side=tk.LEFT)
        gap.pack()

        dev = tk.Frame(frame)
        self.dev_label = tk.Label(dev, width=10, text="deviate:")
        self.dev_label.pack(side=tk.LEFT)
        self.dev_box = tk.Entry(dev, width=15)
        self.dev_box.insert(tk.END, str(self._dev))
        self.dev_box.pack(side=tk.LEFT)
        self.dev_unit = tk.Label(dev, width=5, text="pixel")
        self.dev_unit.pack(side=tk.LEFT)
        dev.pack()

        step = tk.Frame(frame)
        self.step_label = tk.Label(step, width=10, text="step:")
        self.step_label.pack(side=tk.LEFT)
        self.step_box = tk.Entry(step, width=15)
        self.step_box.insert(tk.END, str(self._step))
        self.step_box.pack(side=tk.LEFT)
        self.step_unit = tk.Label(step, width=5, text="pixel")
        self.step_unit.pack(side=tk.LEFT)
        step.pack()

        length = tk.Frame(frame)
        self.length_label = tk.Label(length, width=10, text="length:")
        self.length_label.pack(side=tk.LEFT)
        self.length_box = tk.Entry(length, width=15)
        self.length_box.insert(tk.END, str(self._length))
        self.length_box.pack(side=tk.LEFT)
        self.length_unit = tk.Label(length, width=5, text="pixel")
        self.length_unit.pack(side=tk.LEFT)
        length.pack()

        return frame

    def ok_pressed(self):
        try:
            self._ws = int(self.ws_box.get())
            self._gap = int(self.gap_box.get())
            self._dev = float(self.dev_box.get())
            self._step = float(self.step_box.get())
            self._length = int(self.length_box.get())
        except Exception as e:
            pass
        else:
            self.destroy()

    def cancel_pressed(self):
        self._ws = None
        self._gap = None
        self._dev = None
        self._step = None
        self._length = None
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side="left")
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side="right")
        self.bind("<Return>", lambda event: self.ok_pressed())
        self.bind("<Escape>", lambda event: self.cancel_pressed())

def filterdialog(app, *, initialvalue=('', '', '', '')):
    dialog = FilterDialog(title="Filter", parent=app, initialvalue=initialvalue)
    return dialog._ws, dialog._gap, dialog._dev, dialog._step, dialog._length