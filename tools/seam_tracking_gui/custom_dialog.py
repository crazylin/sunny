import tkinter as tk
from tkinter.simpledialog import Dialog

class DialogDelta(Dialog):
    def __init__(self, parent, title, *, initialvalue: dict):
        self._ok = True
        self._x = initialvalue['delta_x']
        self._y = initialvalue['delta_y']
        super().__init__(parent, title)

    def body(self, frame):
        x = tk.Frame(frame)
        self.x_label = tk.Label(x, width=10, text='Offset x:')
        self.x_label.pack(side=tk.LEFT)
        self.x_box = tk.Entry(x, width=15)
        self.x_box.insert(tk.END, str(self._x) if self._x is not None else '')
        self.x_box.pack(side=tk.LEFT)
        self.x_unit = tk.Label(x, width=5, text='mm')
        self.x_unit.pack(side=tk.LEFT)
        x.pack()

        y = tk.Frame(frame)
        self.y_label = tk.Label(y, width=10, text='Offset y:')
        self.y_label.pack(side=tk.LEFT)
        self.y_box = tk.Entry(y, width=15)
        self.y_box.insert(tk.END, str(self._y) if self._y is not None else '')
        self.y_box.pack(side=tk.LEFT)
        self.y_unit = tk.Label(y, width=5, text='mm')
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
        self._ok = False
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side='left')
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side='right')
        self.bind('<Return>', lambda event: self.ok_pressed())
        self.bind('<Escape>', lambda event: self.cancel_pressed())

def dialog_delta(app, *, initialvalue: dict):
    d = DialogDelta(title='Offset', parent=app, initialvalue=initialvalue)
    return {'delta_x': d._x, 'delta_y': d._y} if d._ok else None

class DialogLineFilter(Dialog):
    def __init__(self, parent, title, *, initialvalue: dict):
        self._ok = True
        self._b = initialvalue['enable']
        self._ws = initialvalue['window_size']
        self._gap = initialvalue['gap']
        self._dev = initialvalue['deviate']
        self._step = initialvalue['step']
        self._length = initialvalue['length']
        super().__init__(parent, title)

    def body(self, frame):
        ws = tk.Frame(frame)
        self.ws_label = tk.Label(ws, width=10, text='window size:', anchor=tk.E)
        self.ws_label.pack(side=tk.LEFT)
        self.ws_box = tk.Entry(ws, width=15)
        self.ws_box.insert(tk.END, str(self._ws) if self._ws is not None else '')
        self.ws_box.pack(side=tk.LEFT)
        self.ws_unit = tk.Label(ws, width=5, text='pixel')
        self.ws_unit.pack(side=tk.LEFT)
        ws.pack()

        gap = tk.Frame(frame)
        self.gap_label = tk.Label(gap, width=10, text='gap:', anchor=tk.E)
        self.gap_label.pack(side=tk.LEFT)
        self.gap_box = tk.Entry(gap, width=15)
        self.gap_box.insert(tk.END, str(self._gap) if self._gap is not None else '')
        self.gap_box.pack(side=tk.LEFT)
        self.gap_unit = tk.Label(gap, width=5, text='pixel')
        self.gap_unit.pack(side=tk.LEFT)
        gap.pack()

        dev = tk.Frame(frame)
        self.dev_label = tk.Label(dev, width=10, text='deviate:', anchor=tk.E)
        self.dev_label.pack(side=tk.LEFT)
        self.dev_box = tk.Entry(dev, width=15)
        self.dev_box.insert(tk.END, str(self._dev) if self._dev is not None else '')
        self.dev_box.pack(side=tk.LEFT)
        self.dev_unit = tk.Label(dev, width=5, text='pixel')
        self.dev_unit.pack(side=tk.LEFT)
        dev.pack()

        step = tk.Frame(frame)
        self.step_label = tk.Label(step, width=10, text='step:', anchor=tk.E)
        self.step_label.pack(side=tk.LEFT)
        self.step_box = tk.Entry(step, width=15)
        self.step_box.insert(tk.END, str(self._step) if self._step is not None else '')
        self.step_box.pack(side=tk.LEFT)
        self.step_unit = tk.Label(step, width=5, text='pixel')
        self.step_unit.pack(side=tk.LEFT)
        step.pack()

        length = tk.Frame(frame)
        self.length_label = tk.Label(length, width=10, text='length:', anchor=tk.E)
        self.length_label.pack(side=tk.LEFT)
        self.length_box = tk.Entry(length, width=15)
        self.length_box.insert(tk.END, str(self._length) if self._length is not None else '')
        self.length_box.pack(side=tk.LEFT)
        self.length_unit = tk.Label(length, width=5, text='pixel')
        self.length_unit.pack(side=tk.LEFT)
        length.pack()

        self.enable = tk.BooleanVar()
        self.enable.set(self._b)
        enable_btn = tk.Checkbutton(frame, width=10, text='Enable filter', variable=self.enable, command=self.toggle)
        enable_btn.pack(side=tk.RIGHT)

        self.toggle()

        return frame

    def ok_pressed(self):
        try:
            self._b = self.enable.get()
            if self._b:
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
        self._ok = False
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side='left')
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side='right')
        self.bind('<Return>', lambda event: self.ok_pressed())
        self.bind('<Escape>', lambda event: self.cancel_pressed())

    def toggle(self):
        if self.enable.get():
            self.ws_box.configure(state=['normal'])
            self.gap_box.configure(state=['normal'])
            self.dev_box.configure(state=['normal'])
            self.step_box.configure(state=['normal'])
            self.length_box.configure(state=['normal'])
        else:
            self.ws_box.configure(state=['disabled'])
            self.gap_box.configure(state=['disabled'])
            self.dev_box.configure(state=['disabled'])
            self.step_box.configure(state=['disabled'])
            self.length_box.configure(state=['disabled'])

def dialog_line_filter(app, *, initialvalue: dict):
    d = DialogLineFilter(title='Laser line filter', parent=app, initialvalue=initialvalue)
    return {
        'enable': d._b,
        'window_size': d._ws,
        'gap': d._gap,
        'deviate': d._dev,
        'step': d._step,
        'length': d._length
        } if d._ok else None

class DialogCenter(Dialog):
    def __init__(self, parent, title, *, initialvalue: dict):
        self._ok = True
        self._ksize = initialvalue['ksize']
        self._threshold = initialvalue['threshold']
        self._wmin = initialvalue['width_min']
        self._wmax = initialvalue['width_max']
        super().__init__(parent, title)

    def body(self, frame):
        ksize = tk.Frame(frame)
        self.ksize_label = tk.Label(ksize, width=10, text='ksize:', anchor=tk.E)
        self.ksize_label.pack(side=tk.LEFT)
        self.ksize_box = tk.Entry(ksize, width=15)
        self.ksize_box.insert(tk.END, str(self._ksize) if self._ksize is not None else '')
        self.ksize_box.pack(side=tk.LEFT)
        self.ksize_unit = tk.Label(ksize, width=15, text='[1, 3, 5, 7, -1]', anchor=tk.W)
        self.ksize_unit.pack(side=tk.LEFT)
        ksize.pack()

        threshold = tk.Frame(frame)
        self.threshold_label = tk.Label(threshold, width=10, text='threshold:', anchor=tk.E)
        self.threshold_label.pack(side=tk.LEFT)
        self.threshold_box = tk.Entry(threshold, width=15)
        self.threshold_box.insert(tk.END, str(self._threshold) if self._threshold is not None else '')
        self.threshold_box.pack(side=tk.LEFT)
        self.threshold_unit = tk.Label(threshold, width=15, text='0~255', anchor=tk.W)
        self.threshold_unit.pack(side=tk.LEFT)
        threshold.pack()

        wmin = tk.Frame(frame)
        self.wmin_label = tk.Label(wmin, width=10, text='width min:', anchor=tk.E)
        self.wmin_label.pack(side=tk.LEFT)
        self.wmin_box = tk.Entry(wmin, width=15)
        self.wmin_box.insert(tk.END, str(self._wmin) if self._wmin is not None else '')
        self.wmin_box.pack(side=tk.LEFT)
        self.wmin_unit = tk.Label(wmin, width=15, text='pixel', anchor=tk.W)
        self.wmin_unit.pack(side=tk.LEFT)
        wmin.pack()

        wmax = tk.Frame(frame)
        self.wmax_label = tk.Label(wmax, width=10, text='width max:', anchor=tk.E)
        self.wmax_label.pack(side=tk.LEFT)
        self.wmax_box = tk.Entry(wmax, width=15)
        self.wmax_box.insert(tk.END, str(self._wmax) if self._wmax is not None else '')
        self.wmax_box.pack(side=tk.LEFT)
        self.wmax_unit = tk.Label(wmax, width=15, text='pixel', anchor=tk.W)
        self.wmax_unit.pack(side=tk.LEFT)
        wmax.pack()

        return frame

    def ok_pressed(self):
        try:
            self._ksize = int(self.ksize_box.get())
            self._threshold = int(self.threshold_box.get())
            self._wmin = int(self.wmin_box.get())
            self._wmax = int(self.wmax_box.get())
        except Exception as e:
            pass
        else:
            self.destroy()

    def cancel_pressed(self):
        self._ok = False
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side='left')
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side='right')
        self.bind('<Return>', lambda event: self.ok_pressed())
        self.bind('<Escape>', lambda event: self.cancel_pressed())

def dialog_center(app, *, initialvalue: dict):
    d = DialogCenter(title='Laser line center', parent=app, initialvalue=initialvalue)
    return {
        'ksize': d._ksize,
        'threshold': d._threshold,
        'width_min': d._wmin,
        'width_max': d._wmax
        } if d._ok else None

class DialogSeamFilter(Dialog):
    def __init__(self, parent, title, *, initialvalue: dict):
        self._ok = True
        self._b = initialvalue['enable']
        self._ws = initialvalue['window_size']
        self._gap = initialvalue['gap']
        self._step = initialvalue['step']
        self._length = initialvalue['length']
        super().__init__(parent, title)

    def body(self, frame):
        ws = tk.Frame(frame)
        self.ws_label = tk.Label(ws, width=10, text='window size:', anchor=tk.E)
        self.ws_label.pack(side=tk.LEFT)
        self.ws_box = tk.Entry(ws, width=15)
        self.ws_box.insert(tk.END, str(self._ws) if self._ws is not None else '')
        self.ws_box.pack(side=tk.LEFT)
        self.ws_unit = tk.Label(ws, width=5, text='pixel')
        self.ws_unit.pack(side=tk.LEFT)
        ws.pack()

        gap = tk.Frame(frame)
        self.gap_label = tk.Label(gap, width=10, text='gap:', anchor=tk.E)
        self.gap_label.pack(side=tk.LEFT)
        self.gap_box = tk.Entry(gap, width=15)
        self.gap_box.insert(tk.END, str(self._gap) if self._gap is not None else '')
        self.gap_box.pack(side=tk.LEFT)
        self.gap_unit = tk.Label(gap, width=5, text='pixel')
        self.gap_unit.pack(side=tk.LEFT)
        gap.pack()

        step = tk.Frame(frame)
        self.step_label = tk.Label(step, width=10, text='step:', anchor=tk.E)
        self.step_label.pack(side=tk.LEFT)
        self.step_box = tk.Entry(step, width=15)
        self.step_box.insert(tk.END, str(self._step) if self._step is not None else '')
        self.step_box.pack(side=tk.LEFT)
        self.step_unit = tk.Label(step, width=5, text='pixel')
        self.step_unit.pack(side=tk.LEFT)
        step.pack()

        length = tk.Frame(frame)
        self.length_label = tk.Label(length, width=10, text='length:', anchor=tk.E)
        self.length_label.pack(side=tk.LEFT)
        self.length_box = tk.Entry(length, width=15)
        self.length_box.insert(tk.END, str(self._length) if self._length is not None else '')
        self.length_box.pack(side=tk.LEFT)
        self.length_unit = tk.Label(length, width=5, text='pixel')
        self.length_unit.pack(side=tk.LEFT)
        length.pack()

        self.enable = tk.BooleanVar()
        self.enable.set(self._b)
        enable_btn = tk.Checkbutton(frame, width=10, text='Enable filter', variable=self.enable, command=self.toggle)
        enable_btn.pack(side=tk.RIGHT)

        self.toggle()

        return frame

    def ok_pressed(self):
        try:
            self._b = self.enable.get()
            if self._b:
                self._ws = int(self.ws_box.get())
                self._gap = int(self.gap_box.get())
                self._step = float(self.step_box.get())
                self._length = int(self.length_box.get())
        except Exception as e:
            pass
        else:
            self.destroy()

    def cancel_pressed(self):
        self._ok = False
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side='left')
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side='right')
        self.bind('<Return>', lambda event: self.ok_pressed())
        self.bind('<Escape>', lambda event: self.cancel_pressed())

    def toggle(self):
        if self.enable.get():
            self.ws_box.configure(state=['normal'])
            self.gap_box.configure(state=['normal'])
            self.step_box.configure(state=['normal'])
            self.length_box.configure(state=['normal'])
        else:
            self.ws_box.configure(state=['disabled'])
            self.gap_box.configure(state=['disabled'])
            self.step_box.configure(state=['disabled'])
            self.length_box.configure(state=['disabled'])

def dialog_seam_filter(app, *, initialvalue: dict):
    d = DialogSeamFilter(title='Seam tracking filter', parent=app, initialvalue=initialvalue)
    return {
        'enable': d._b,
        'window_size': d._ws,
        'gap': d._gap,
        'step': d._step,
        'length': d._length
        } if d._ok else None