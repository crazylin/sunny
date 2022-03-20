import tkinter as tk
from tkinter.simpledialog import Dialog

class MyDialog(Dialog):
    def __init__(self, parent, title):
        self.delta_x = None
        self.delta_y = None
        super().__init__(parent, title)

    def body(self, frame):
        # print(type(frame)) # tkinter.Frame
        u = tk.Frame(frame)
        self.delta_x_label = tk.Label(u, width=10, text="Offset x:")
        self.delta_x_label.pack(side=tk.LEFT)
        self.delta_x_box = tk.Entry(u, width=15)
        self.delta_x_box.pack(side=tk.LEFT)
        self.delta_x_label_mm = tk.Label(u, width=5, text="mm")
        self.delta_x_label_mm.pack(side=tk.LEFT)
        u.pack()

        d = tk.Frame(frame)
        self.delta_y_label = tk.Label(d, width=10, text="Offset y:")
        self.delta_y_label.pack(side=tk.LEFT)
        self.delta_y_box = tk.Entry(d, width=15)
        self.delta_y_box.pack(side=tk.LEFT)
        self.delta_y_label_mm = tk.Label(d, width=5, text="mm")
        self.delta_y_label_mm.pack(side=tk.LEFT)
        d.pack()

        return frame

    def ok_pressed(self):
        # print("ok")
        try:
            self.delta_x = float(self.delta_x_box.get())
            self.delta_y = float(self.delta_y_box.get())
        except Exception as e:
            pass
        else:
            self.destroy()

    def cancel_pressed(self):
        self.destroy()

    def buttonbox(self):
        self.ok_button = tk.Button(self, text='OK', width=5, command=self.ok_pressed)
        self.ok_button.pack(side="left")
        cancel_button = tk.Button(self, text='Cancel', width=5, command=self.cancel_pressed)
        cancel_button.pack(side="right")
        self.bind("<Return>", lambda event: self.ok_pressed())
        self.bind("<Escape>", lambda event: self.cancel_pressed())

def mydialog(app):
    dialog = MyDialog(title="Offset", parent=app)
    return dialog.delta_x, dialog.delta_y