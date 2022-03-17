import rclpy
import tkinter as tk
from point_data import PointData
from custom_figure import CustomFigure
from tkinter import ttk
from tkinter import messagebox
from ros_node import RosNode

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)

from threading import Thread


class App(tk.Tk):
    """Toplevel window."""

    def __init__(self):
        super().__init__()

        self.line_data = PointData()
        self.pick_data = PointData()
        self.code = ''
        self.title('Seam Tracking')
        self.option_add('*tearOff', False)

        self.__initMenu()

        frameL = self.__initPlot()
        frameL.grid(row=0, column=0, sticky=tk.NSEW)

        frameR = self.__initList()
        frameR.grid(row=0, column=1, sticky=tk.NSEW)

        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1)

        self.protocol("WM_DELETE_WINDOW", self.__exit)

        self.ros = RosNode()
        self.ros.sub_line(self._callbackLine)
        self.ros.sub_pick(self._callbackPick)

        self._thread = Thread(target=self.ros.spin)
        self._thread.start()

    def __exit(self):
        self.ros.destroy_node()
        self.destroy()

    def __initPlot(self):
        frame = ttk.Frame(self)

        fig = CustomFigure()

        canvas = FigureCanvasTkAgg(fig, master=frame)

        tool = ttk.Frame(frame)
        NavigationToolbar2Tk(canvas, tool)

        frame.grid(row=0, column=0, sticky=tk.NSEW)
        canvas.get_tk_widget().grid(row=0, column=0, sticky=tk.NSEW)
        tool.grid(row=1, column=0, sticky=tk.W)

        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.bind('<<RosSubLine>>', lambda e: fig.update_line(self.line_data))
        self.bind('<<RosSubLine>>', lambda e: canvas.draw_idle(), add='+')
        self.bind('<<RosSubPick>>', lambda e: fig.update_pick(self.pick_data))
        self.bind('<<RosSubPick>>', lambda e: canvas.draw_idle(), add='+')
        return frame

    def __initList(self):
        frame = ttk.Frame(self, padding=(10,0,10,0))

        label = ttk.Label(frame, text='Configuration:')
        texts = tk.Text(frame, wrap = 'none')
        btnAdd = ttk.Button(frame, text='Add', width=10, command=self._btnGetCode)
        btnRemove = ttk.Button(frame, text='Remove', width=10)
        btnUpload = ttk.Button(frame, text='Upload', width=10)

        frame.grid(row=0, column=0, sticky=tk.NSEW)
        label.grid(row=0, column=0, columnspan=3, sticky=tk.W)
        texts.grid(row=1, column=0, columnspan=3, sticky=tk.NSEW)
        btnAdd.grid(row=2, column=0, sticky=tk.EW)
        btnRemove.grid(row=2, column=1, sticky=tk.EW)
        btnUpload.grid(row=2, column=2, sticky=tk.EW)

        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

        self.bind('<<UpdateCode>>', lambda e: texts.replace(1.0, "end", self.code))
        return frame

    def __initMenu(self):
        menubar = tk.Menu(self)
        self['menu'] = menubar

        menu_file = tk.Menu(menubar)
        menu_file.add_command(label='New')
        menu_file.add_command(label='Open...')
        menu_file.add_command(label='Close')

        menu_edit = tk.Menu(menubar)
        menu_help = tk.Menu(menubar)

        menubar.add_cascade(menu=menu_file, label='File')
        menubar.add_cascade(menu=menu_edit, label='Edit')
        menubar.add_cascade(menu=menu_help, label='Help')

    def _callbackLine(self, msg):
        self.line_data.from_msg(msg, u = 'y', v = 'z')
        self.event_generate('<<RosSubLine>>', when='tail')

    def _callbackPick(self, msg):
        self.pick_data.from_msg(msg, u = 'y', v = 'z')
        self.event_generate('<<RosSubPick>>', when='tail')

    def _btnGetCode(self, *args):
        self.future = self.ros.get_code()
        if self.future != None:
            self.future.add_done_callback(self._btnGetCodeDone)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _btnGetCodeDone(self, future):
        try:
            res = future.result()
            if res.success:
                self.code = res.code
            else:
                self.code = res.message
        except Exception as e:
            self.code = str(e)
        self.event_generate('<<UpdateCode>>', when='tail')

    def _btnSetCode(self, *args):
        self.future = self.ros.get_code()
        if self.future != None:
            self.future.add_done_callback(self._btnGetCodeDone)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _btnSetCodeDone(self, future):
        try:
            res = future.result()
            if res.success:
                self.code = res.code
            else:
                self.code = res.message
        except Exception as e:
            self.code = str(e)
        self.event_generate('<<UpdateCode>>', when='tail')

    def _btnGetCodes(self, *args):
        self.future = self.ros.get_code()
        if self.future != None:
            self.future.add_done_callback(self._btnGetCodeDone)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _btnGetCodeDone(self, future):
        try:
            res = future.result()
            if res.success:
                self.code = res.code
            else:
                self.code = res.message
        except Exception as e:
            self.code = str(e)
        self.event_generate('<<UpdateCode>>', when='tail')

if __name__ == '__main__':
    rclpy.init()

    app = App()

    app.mainloop()

    rclpy.shutdown()
