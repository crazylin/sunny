from email import message
import rclpy
import tkinter as tk
from point_data import PointData
from custom_figure import CustomFigure
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog
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

        self._init_menu()

        frameL = self._init_plot()
        frameL.grid(row=0, column=0, sticky=tk.NSEW)

        frameR = self._init_list()
        frameR.grid(row=0, column=1, sticky=tk.NSEW)

        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=1)

        self.protocol("WM_DELETE_WINDOW", self.__exit)

        self.ros = RosNode()
        self.ros.sub_line(self._ros_cb_line)
        self.ros.sub_pick(self._ros_cb_pick)

        self._thread = Thread(target=self.ros.spin)
        self._thread.start()

    def __exit(self):
        self.ros.destroy_node()
        self.destroy()

    def _init_plot(self):
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

    def _init_list(self):
        frame = ttk.Frame(self, padding=(10,0,10,0))

        label = ttk.Label(frame, text='Configuration:')
        self.texts = tk.Text(frame, wrap = 'none')
        btn_pull = ttk.Button(frame, text='Pull', width=10, command=self._cb_btn_get_code)
        btn_push = ttk.Button(frame, text='Push', width=10, command=self._cb_btn_set_code)
        btnAdd = ttk.Button(frame, text='Add', width=10)
        btnRemove = ttk.Button(frame, text='Remove', width=10)
        btn_backup = ttk.Button(frame, text='Backup...', width=10, command=self._cb_btn_get_codes)
        btn_upload = ttk.Button(frame, text='Upload...', width=10, command=self._cb_btn_set_codes)
        self.btn_laser = ttk.Button(frame, text='Laser on', width=10, command=self._cb_btn_laser)
        self.btn_camera = ttk.Button(frame, text='Camera on', width=10, command=self._cb_btn_camera)

        frame.grid(row=0, column=0, sticky=tk.NSEW)
        label.grid(row=0, column=0, columnspan=4, sticky=tk.W)
        self.texts.grid(row=1, column=0, columnspan=4, sticky=tk.NSEW)

        self.btn_laser.grid(row=2, column=0, sticky=tk.EW)
        self.btn_camera.grid(row=3, column=0, sticky=tk.EW)

        btn_pull.grid(row=2, column=1, sticky=tk.EW)
        btn_push.grid(row=3, column=1, sticky=tk.EW)

        btnAdd.grid(row=2, column=2, sticky=tk.EW)
        btnRemove.grid(row=3, column=2, sticky=tk.EW)

        btn_backup.grid(row=2, column=3, sticky=tk.EW)
        btn_upload.grid(row=3, column=3, sticky=tk.EW)

        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.columnconfigure(3, weight=1)

        self.bind('<<UpdateCode>>', lambda e: self.texts.replace(1.0, "end", self.code))
        return frame

    def _init_menu(self):
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

    def _ros_cb_line(self, msg):
        self.line_data.from_msg(msg, u='y', v='z')
        self.event_generate('<<RosSubLine>>', when='tail')

    def _ros_cb_pick(self, msg):
        self.pick_data.from_msg(msg, u='y', v='z')
        self.event_generate('<<RosSubPick>>', when='tail')

    def _cb_btn_laser(self):
        if self.btn_laser['text'] == 'Laser on':
            future = self.ros.laser_on()
            if future != None:
                self.btn_laser.state(['pressed'])
                future.add_done_callback(self._cb_btn_laser_on_done)
            else:
                messagebox.showinfo(message='Service is not ready!')
        else:
            future = self.ros.laser_off()
            if future != None:
                self.btn_laser.state(['!pressed'])
                future.add_done_callback(self._cb_btn_laser_off_done)
            else:
                messagebox.showinfo(message='Service is not ready!')
    
    def _cb_btn_laser_on_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_laser['text'] = 'Laser off'
            else:
                messagebox.showwarning(message=res.message)
                self.btn_laser.state(['!pressed'])
        except Exception as e:
            messagebox.showerror(message=str(e))
            self.btn_laser.state(['!pressed'])

    def _cb_btn_laser_off_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_laser['text'] = 'Laser on'
            else:
                messagebox.showwarning(message=res.message)
                self.btn_laser.state(['pressed'])
        except Exception as e:
            messagebox.showerror(message=str(e))
            self.btn_laser.state(['pressed'])

    def _cb_btn_camera(self):
        if self.btn_camera['text'] == 'Camera on':
            future = self.ros.camera_on()
            if future != None:
                self.btn_camera.state(['pressed'])
                future.add_done_callback(self._cb_btn_camera_on_done)
            else:
                messagebox.showinfo(message='Service is not ready!')
        else:
            future = self.ros.camera_off()
            if future != None:
                self.btn_camera.state(['!pressed'])
                future.add_done_callback(self._cb_btn_camera_off_done)
            else:
                messagebox.showinfo(message='Service is not ready!')
    
    def _cb_btn_camera_on_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_camera['text'] = 'Camera off'
            else:
                messagebox.showwarning(message=res.message)
                self.btn_camera.state(['!pressed'])
        except Exception as e:
            messagebox.showerror(message=str(e))
            self.btn_camera.state(['!pressed'])

    def _cb_btn_camera_off_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_camera['text'] = 'Camera on'
            else:
                messagebox.showwarning(message=res.message)
                self.btn_camera.state(['pressed'])
        except Exception as e:
            messagebox.showerror(message=str(e))
            self.btn_camera.state(['pressed'])

    def _cb_btn_get_code(self, *args):
        future = self.ros.get_code()
        if future != None:
            future.add_done_callback(self._cb_btn_get_code_done)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _cb_btn_get_code_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.code = res.code
                self.event_generate('<<UpdateCode>>', when='tail')
            else:
                messagebox.showwarning(message=res.message)
        except Exception as e:
            messagebox.showerror(message=str(e))

    def _cb_btn_set_code(self, *args):
        self.code = self.texts.get('1.0', 'end')
        future = self.ros.set_code(self.code)
        if future != None:
            future.add_done_callback(self._cb_btn_set_code_done)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _cb_btn_set_code_done(self, future):
        try:
            res = future.result()
            if res.success:
                messagebox.showinfo(message='Done!')
            else:
                messagebox.showwarning(message=res.message)
        except Exception as e:
            messagebox.showerror(message=str(e))

    def _cb_btn_get_codes(self, *args):
        future = self.ros.get_codes()
        if future != None:
            future.add_done_callback(self._cb_btn_get_codes_done)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _cb_btn_get_codes_done(self, future):
        try:
            res = future.result()
            if res.success:
                filename = filedialog.asksaveasfilename(initialfile='codes.json', defaultextension='json')
                if filename:
                    with open(filename, 'w') as fp:
                        fp.write(res.codes)
            else:
                messagebox.showwarning(message=res.message)
        except Exception as e:
            messagebox.showerror(message=str(e))

    def _cb_btn_set_codes(self, *args):
        filename = filedialog.askopenfilename(defaultextension='json')
        if not filename:
            return

        with open(filename, 'r') as fp:
            codes = fp.read()

        future = self.ros.set_codes(codes)
        if future != None:
            future.add_done_callback(self._cb_btn_set_codes_done)
        else:
            messagebox.showinfo(message='Service is not ready!')

    def _cb_btn_set_codes_done(self, future):
        try:
            res = future.result()
            if res.success:
                messagebox.showinfo(message='Done!')
            else:
                messagebox.showwarning(message=res.message)
        except Exception as e:
            messagebox.showerror(message=str(e))

if __name__ == '__main__':
    rclpy.init()

    app = App()

    app.mainloop()

    rclpy.shutdown()
