import json
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

        self.pnts_data = PointData()
        self.seam_data = PointData()
        self.codes = ['def fn(x: list, y: list):\n    return [], []']
        self.index = 0
        self.title('Seam Tracking GUI')
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
        self.ros.sub_pnts(self._ros_cb_pnts)
        self.ros.sub_seam(self._ros_cb_seam)

        self._thread = Thread(target=rclpy.spin, args=[self.ros])
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

        self.bind('<<RosSubLine>>', lambda e: fig.update_pnts(self.pnts_data))
        self.bind('<<RosSubLine>>', lambda e: canvas.draw_idle(), add='+')
        self.bind('<<RosSubPick>>', lambda e: fig.update_seam(self.seam_data))
        self.bind('<<RosSubPick>>', lambda e: canvas.draw_idle(), add='+')
        return frame

    def _init_list(self):
        frame = ttk.Frame(self, padding=(10,0,10,0))

        self.label = ttk.Label(frame, text='Task:', width=10)
        self.btn_previous = ttk.Button(frame, text='Previous', width=10, command=self._cb_btn_previous)
        self.btn_next = ttk.Button(frame, text='Next', width=10, command=self._cb_btn_next)
        self.btn_refresh = ttk.Button(frame, text='Refresh', width=10, command=self._cb_btn_refresh)

        self.texts = tk.Text(frame, wrap = 'none')

        self.btn_laser = ttk.Button(frame, text='Laser on', width=10, command=self._cb_btn_laser)
        self.btn_camera = ttk.Button(frame, text='Camera on', width=10, command=self._cb_btn_camera)

        self.btn_add = ttk.Button(frame, text='Add', width=10, command=self._cb_btn_add)
        self.btn_del = ttk.Button(frame, text='Delete', width=10, command=self._cb_btn_del)

        self.btn_push = ttk.Button(frame, text='Push', width=10, command=self._cb_btn_push)
        self.btn_commit = ttk.Button(frame, text='Commit', width=10, command=self._cb_btn_commit)

        self.btn_backup = ttk.Button(frame, text='Backup...', width=10, command=self._cb_btn_backup)
        self.btn_upload = ttk.Button(frame, text='Upload...', width=10, command=self._cb_btn_upload)

        frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.label.grid(row=0, column=0, columnspan=1, sticky=tk.EW)
        self.btn_previous.grid(row=0, column=1, columnspan=1, sticky=tk.EW)
        self.btn_next.grid(row=0, column=2, columnspan=1, sticky=tk.EW)
        self.btn_refresh.grid(row=0, column=3, columnspan=1, sticky=tk.EW)

        self.texts.grid(row=1, column=0, columnspan=4, sticky=tk.NSEW)

        self.btn_laser.grid(row=2, column=0, sticky=tk.EW)
        self.btn_camera.grid(row=3, column=0, sticky=tk.EW)

        self.btn_add.grid(row=2, column=1, sticky=tk.EW)
        self.btn_del.grid(row=3, column=1, sticky=tk.EW)

        self.btn_push.grid(row=3, column=2, sticky=tk.EW)
        self.btn_commit.grid(row=2, column=2, sticky=tk.EW)

        self.btn_backup.grid(row=2, column=3, sticky=tk.EW)
        self.btn_upload.grid(row=3, column=3, sticky=tk.EW)

        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.columnconfigure(3, weight=1)

        self.bind('<<UpdateCode>>', lambda e: self.texts.replace(1.0, "end", self.codes[self.index]))
        self._change_index(self.index)
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

    def _ros_cb_pnts(self, msg):
        self.pnts_data.from_msg(msg)
        self.event_generate('<<RosSubLine>>', when='tail')

    def _ros_cb_seam(self, msg):
        self.seam_data.from_msg(msg)
        self.event_generate('<<RosSubPick>>', when='tail')

    def _cb_btn_laser(self, *args):
        if self.btn_laser['text'] == 'Laser on':
            future = self.ros.laser_on()
            if future is not None:
                self.btn_laser.state(['pressed'])
                future.add_done_callback(self._cb_btn_laser_on_done)
            else:
                messagebox.showinfo('Info', message='Service is not ready!')
        else:
            future = self.ros.laser_off()
            if future is not None:
                self.btn_laser.state(['!pressed'])
                future.add_done_callback(self._cb_btn_laser_off_done)
            else:
                messagebox.showinfo('Info', message='Service is not ready!')
    
    def _cb_btn_laser_on_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_laser['text'] = 'Laser off'
            else:
                messagebox.showwarning('Warning', message=res.message)
                self.btn_laser.state(['!pressed'])
        except Exception as e:
            messagebox.showerror('Error', message=str(e))
            self.btn_laser.state(['!pressed'])

    def _cb_btn_laser_off_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_laser['text'] = 'Laser on'
            else:
                messagebox.showwarning('Warning', message=res.message)
                self.btn_laser.state(['pressed'])
        except Exception as e:
            messagebox.showerror('Error', message=str(e))
            self.btn_laser.state(['pressed'])

    def _cb_btn_camera(self, *args):
        if self.btn_camera['text'] == 'Camera on':
            future = self.ros.camera_on()
            if future is not None:
                self.btn_camera.state(['pressed'])
                future.add_done_callback(self._cb_btn_camera_on_done)
            else:
                messagebox.showinfo('Info', message='Service is not ready!')
        else:
            future = self.ros.camera_off()
            if future is not None:
                self.btn_camera.state(['!pressed'])
                future.add_done_callback(self._cb_btn_camera_off_done)
            else:
                messagebox.showinfo('Info', message='Service is not ready!')
    
    def _cb_btn_camera_on_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_camera['text'] = 'Camera off'
            else:
                messagebox.showwarning('Warning', message=res.message)
                self.btn_camera.state(['!pressed'])
        except Exception as e:
            messagebox.showerror('Error', message=str(e))
            self.btn_camera.state(['!pressed'])

    def _cb_btn_camera_off_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.btn_camera['text'] = 'Camera on'
            else:
                messagebox.showwarning('Warning', message=res.message)
                self.btn_camera.state(['pressed'])
        except Exception as e:
            messagebox.showerror('Error', message=str(e))
            self.btn_camera.state(['pressed'])

    # def _cb_btn_get_code(self, *args):
    #     future = self.ros.get_code()
    #     if future is not None:
    #         future.add_done_callback(self._cb_btn_get_code_done)
    #     else:
    #         messagebox.showinfo('Info', message='Service is not ready!')

    # def _cb_btn_get_code_done(self, future):
    #     try:
    #         res = future.result()
    #         if res.success:
    #             self.code = res.code
    #             self.event_generate('<<UpdateCode>>', when='tail')
    #         else:
    #             messagebox.showwarning('Warning', message=res.message)
    #     except Exception as e:
    #         messagebox.showerror('Error', message=str(e))

    # def _cb_btn_set_code(self, *args):
    #     self.code = self.texts.get('1.0', 'end')
    #     future = self.ros.set_code(self.code)
    #     if future is not None:
    #         future.add_done_callback(self._cb_btn_set_code_done)
    #     else:
    #         messagebox.showinfo('Info', message='Service is not ready!')

    # def _cb_btn_set_code_done(self, future):
    #     try:
    #         res = future.result()
    #         if res.success:
    #             messagebox.showinfo('Info', message='Done!')
    #         else:
    #             messagebox.showwarning('Warning', message=res.message)
    #     except Exception as e:
    #         messagebox.showerror('Error', message=str(e))

    def _cb_btn_backup(self, *args):
        filename = filedialog.asksaveasfilename(
            title='Backup codes',
            initialfile='codes.json',
            defaultextension='json',
            filetypes=[('JSON JavaScript Object Notation', '.json')])
        if filename:
            with open(filename, 'w') as fp:
                json.dump(self.codes[1:], fp)

    def _cb_btn_upload(self, *args):
        filename = filedialog.askopenfilename(
            title='Upload codes',
            initialfile='codes.json',
            defaultextension='json',
            filetypes=[('JSON JavaScript Object Notation', '.json')])
        if filename:
            with open(filename, 'r') as fp:
                self.codes[1:] = json.load(fp)
            self._update_codes()

    def _cb_btn_refresh(self, *args):
        future = self.ros.get_codes()
        if future is not None:
            future.add_done_callback(self._cb_btn_refresh_done)
        else:
            messagebox.showinfo('Info', message='Service is not ready!')

    def _cb_btn_refresh_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.codes[1:] = json.loads(res.codes)
                self._update_codes()
            else:
                messagebox.showwarning('Warning', message=res.message)
        except Exception as e:
            messagebox.showerror('Error', message=str(e))

    def _cb_btn_previous(self, *args):
        if self._code_modified():
            answer = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if answer:
                self._change_index(self.index - 1)
        else:
            self._change_index(self.index - 1)

    def _cb_btn_next(self, *args):
        if self.index !=0 and self._code_modified():
            answer = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if answer:
                self._change_index(self.index + 1)
        else:
            self._change_index(self.index + 1)

    def _cb_btn_add(self, *args):
        self.codes.append('def fn(x: list, y: list):\n    return [], []')
        self._change_index(len(self.codes) - 1)

    def _cb_btn_del(self, *args):
        del self.codes[self.index]
        self._update_codes()

    def _cb_btn_push(self, *args):
        s = json.dumps(self.codes[1:])
        future = self.ros.set_codes(s)
        if future is not None:
            future.add_done_callback(self._cb_btn_push_done)
        else:
            messagebox.showinfo('Info', message='Service is not ready!')

    def _cb_btn_push_done(self, future):
        try:
            res = future.result()
            if res.success:
                messagebox.showinfo('Info', message='Done!')
            else:
                messagebox.showwarning('Warning', message=res.message)
        except Exception as e:
            messagebox.showerror('Error', message=str(e))

    def _cb_btn_commit(self, *args):
        if self._code_modified():
            self.codes[self.index] = self.texts.get('1.0', 'end').rstrip()
            messagebox.showinfo('Info', message='Done!')
        else:
            messagebox.showinfo('Info', message='Nothing modified!')

    def _update_codes(self):
        if self.index > len(self.codes) - 1:
            self.index = len(self.codes) - 1
        self._change_index(self.index)

    def _change_index(self, index):
        if index < 0 or index > len(self.codes) - 1:
            return
        self.index = index
        self.label['text'] = f'Task: {index:>2}'
        if index == 0:
            self.btn_previous.state(['disabled'])
            self.btn_del.state(['disabled'])
        else:
            self.btn_previous.state(['!disabled'])
            self.btn_del.state(['!disabled'])
        if index == len(self.codes) - 1:
            self.btn_next.state(['disabled'])
        else:
            self.btn_next.state(['!disabled'])
        self.event_generate('<<UpdateCode>>', when='tail')

    def _code_modified(self):
        if self.codes[self.index].rstrip() == self.texts.get('1.0', 'end').rstrip():
            return False
        else:
            return True

if __name__ == '__main__':
    rclpy.init()

    app = App()

    app.mainloop()

    rclpy.shutdown()
