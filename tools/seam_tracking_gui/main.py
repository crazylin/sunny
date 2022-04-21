#!/usr/bin/env python3

# Copyright 2019 Zhushi Tech, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import json
import yaml
import tkinter as tk
from threading import Thread
from threading import Lock
from tkinter import ttk, simpledialog, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText
from ros_node import RosNode, from_parameter_value
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from custom_figure import CustomFigure
from custom_dialog import dialog_delta, dialog_center, dialog_line_filter, dialog_seam_filter
from datetime import datetime


class App(tk.Tk):
    """Toplevel window."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = Lock()
        self._task = 0

        self._params = {
            'camera_tis_node': {'exposure_time': 1000, 'power': False},
            'gpio_raspberry_node': {'laser': False},
            'seam_tracking_node': {
                'codes': [],
                'task': -1,
                'delta_x': 0.,
                'delta_y': 0.,
                'enable': False,
                'window_size': 10,
                'gap': 2,
                'step': 2.,
                'length': 5},
            'laser_line_center_node': {
                'ksize': 5,
                'threshold': 35,
                'width_min': 1.,
                'width_max': 30.},
            'laser_line_filter_node': {
                'enable': False,
                'window_size': 10,
                'gap': 5,
                'deviate': 5.,
                'step': 2.,
                'length': 30}
        }

        self._params_cb = {
            'camera_tis_node': {'power': self._params_cb_power},
            'gpio_raspberry_node': {'laser': self._params_cb_laser},
            'seam_tracking_node': {'task': self._params_cb_task}
        }

        self.title('Seam Tracking GUI')
        self.option_add('*tearOff', False)
        self.protocol("WM_DELETE_WINDOW", self.__exit)

        self._init_menu()

        frameL = self._init_plot()
        frameL.grid(row=0, column=0, sticky=tk.NSEW)

        frameR = self._init_list()
        frameR.grid(row=0, column=1, sticky=tk.NSEW)

        self.status = ScrolledText(self, height=5, wrap='none')
        self.status.grid(row=1, column=0, columnspan=2, sticky=tk.EW)

        self.rowconfigure(0, weight=4)
        self.rowconfigure(1, weight=1)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=3)

        self.ros = RosNode(self._params)
        # self.ros.sub_pnts(self._ros_cb_pnts)
        self.ros.sub_seam(self._ros_cb_seam)
        self.ros.sub_log(self._ros_cb_log)

        self._thread = Thread(target=rclpy.spin, args=[self.ros])
        self._thread.start()

        self._refresh()

        self._update_codes()

    def __exit(self):
        self.ros.destroy_node()
        self.destroy()

    def _init_plot(self):
        frame = ttk.Frame(self)

        self.fig = CustomFigure()

        canvas = FigureCanvasTkAgg(self.fig, master=frame)

        tool = ttk.Frame(frame)
        NavigationToolbar2Tk(canvas, tool)

        frame.grid(row=0, column=0, sticky=tk.NSEW)
        canvas.get_tk_widget().grid(row=0, column=0, sticky=tk.NSEW)
        tool.grid(row=1, column=0, sticky=tk.W)

        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        # self.bind('<<RosSubPnts>>', self.fig.update_pnts)
        # self.bind('<<RosSubPnts>>', lambda e: canvas.draw_idle(), add='+')
        self.bind('<<RosSubSeam>>', self.fig.update_seam)
        self.bind('<<RosSubSeam>>', lambda e: canvas.draw_idle(), add='+')
        return frame

    def _init_list(self):
        frame = ttk.Frame(self, padding=(10, 0, 10, 0))

        self.btn_task = ttk.Button(
            frame,
            text='Task:',
            width=10,
            command=self._cb_btn_task)

        self.btn_previous = ttk.Button(
            frame,
            text='Previous',
            width=10,
            command=self._cb_btn_previous)

        self.btn_next = ttk.Button(
            frame,
            text='Next',
            width=10,
            command=self._cb_btn_next)

        self.btn_refresh = ttk.Button(
            frame,
            text='Refresh',
            width=10,
            command=self._cb_btn_refresh)

        self.texts = ScrolledText(frame, wrap='none')

        self.btn_laser = ttk.Button(
            frame,
            text='Laser on',
            width=10,
            command=self._cb_btn_laser)

        self.btn_power = ttk.Button(
            frame,
            text='Camera on',
            width=10,
            command=self._cb_btn_power)

        self.btn_append = ttk.Button(
            frame,
            text='Append',
            width=10,
            command=self._cb_btn_append)

        self.btn_delete = ttk.Button(
            frame,
            text='Delete',
            width=10,
            command=self._cb_btn_delete)

        self.btn_modify = ttk.Button(
            frame,
            text='Modify',
            width=10,
            command=self._cb_btn_modify)

        self.btn_commit = ttk.Button(
            frame,
            text='Commit',
            width=10,
            command=self._cb_btn_commit)

        self.btn_backup = ttk.Button(
            frame,
            text='Backup...',
            width=10,
            command=self._cb_btn_backup)

        self.btn_upload = ttk.Button(
            frame,
            text='Upload...',
            width=10,
            command=self._cb_btn_upload)

        frame.grid(row=0, column=0, sticky=tk.NSEW)

        self.btn_task.grid(row=0, column=0, columnspan=1, sticky=tk.EW)
        self.btn_previous.grid(row=0, column=1, columnspan=1, sticky=tk.EW)
        self.btn_next.grid(row=0, column=2, columnspan=1, sticky=tk.EW)
        self.btn_refresh.grid(row=0, column=3, columnspan=1, sticky=tk.EW)

        self.texts.grid(row=1, column=0, columnspan=4, sticky=tk.NSEW)

        self.btn_laser.grid(row=2, column=0, sticky=tk.EW)
        self.btn_power.grid(row=3, column=0, sticky=tk.EW)

        self.btn_append.grid(row=2, column=1, sticky=tk.EW)
        self.btn_delete.grid(row=3, column=1, sticky=tk.EW)

        self.btn_commit.grid(row=3, column=2, sticky=tk.EW)
        self.btn_modify.grid(row=2, column=2, sticky=tk.EW)

        self.btn_backup.grid(row=2, column=3, sticky=tk.EW)
        self.btn_upload.grid(row=3, column=3, sticky=tk.EW)

        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)
        frame.columnconfigure(3, weight=1)

        return frame

    def _init_menu(self):
        menubar = tk.Menu(self)
        self['menu'] = menubar

        menu_file = tk.Menu(menubar)
        menu_file.add_command(label='New', command=lambda: self.btn_append.invoke())
        menu_file.add_command(label='Open...', command=lambda: self.btn_upload.invoke())
        menu_file.add_command(label='Save...', command=lambda: self.btn_backup.invoke())
        menu_file.add_command(label='Close', command=self.__exit)

        menu_edit = tk.Menu(menubar)
        menu_edit.add_command(label='Exposure time...', command=self._cb_menu_exposure)
        menu_edit.add_command(label='Offset...', command=self._cb_menu_offset)
        menu_edit.add_command(label='Center...', command=self._cb_menu_center)
        menu_edit.add_command(label='Line filter...', command=self._cb_menu_line_filter)
        menu_edit.add_command(label='Seam filter...', command=self._cb_menu_seam_filter)
        menu_edit.add_command(label='Preserve config', command=self._cb_menu_preserve_config)
        menu_edit.add_command(label='Reboot defaults', command=self._cb_menu_reboot_defaults)

        menu_help = tk.Menu(menubar)

        menubar.add_cascade(menu=menu_file, label='File')
        menubar.add_cascade(menu=menu_edit, label='Edit')
        menubar.add_cascade(menu=menu_help, label='Help')

    # def _ros_cb_pnts(self, msg):
    #     self.fig.msg_to_pnts(msg)
    #     self.event_generate('<<RosSubPnts>>', when='tail')

    def _ros_cb_seam(self, msg):
        self.fig.msg_to_seam(msg)
        self.event_generate('<<RosSubSeam>>', when='tail')

    def _ros_cb_log(self, msg):
        if msg.level == 10:
            level = 'Debug'
        elif msg.level == 20:
            level = 'Info'
        elif msg.level == 30:
            level = 'Warn'
        elif msg.level == 40:
            level = 'Error'
        elif msg.level == 50:
            level = 'Fatal'
        self._msg(msg.name + ': ' + msg.msg, level=level)

    def _cb_set_params_done(self, future, dd: dict):
        try:
            (n, d), = dd.items()
            res = future.result().results
            for r, k in zip(res, d):
                if r.successful:
                    if n in self._params_cb and k in self._params_cb[n]:
                        self._params_cb[n][k](d[k])
                    if self._params[n][k] != d[k]:
                        self._params[n][k] = d[k]
                        self._msg(f'[{n}] [{k}] set to: {d[k]}')
                else:
                    self._msg(f'[{n}] [{k}] failed: {r.reason}', level='Warn')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _cb_get_params_done(self, future, dl: dict):
        try:
            (n, l), = dl.items()
            values = future.result().values
            for p, k in zip(values, l):
                v = from_parameter_value(p)
                if n in self._params_cb and k in self._params_cb[n]:
                    self._params_cb[n][k](v)
                if self._params[n][k] != v:
                    self._params[n][k] = v
                    self._msg(f'[{n}] [{k}] set to: {v}')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _cb_menu_exposure(self, *args):
        v = self._params['camera_tis_node']['exposure_time']
        v = str(v) if v is not None else ''
        exp = simpledialog.askinteger('Exposure time', 'Input exposure time:', initialvalue=v)
        if exp is None:
            return
        d = {'exposure_time': exp}
        future = self.ros.set_params('camera_tis_node', d)
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'camera_tis_node': d}))
        else:
            self._msg('Service [camera_tis_node] is not ready!', level='Warn')

    def _cb_menu_offset(self, *args):
        d = dialog_delta(self, initialvalue=self._params['seam_tracking_node'])
        if d is None:
            return
        future = self.ros.set_params('seam_tracking_node', d)
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'seam_tracking_node': d}))
        else:
            self._msg('Service [seam_tracking_node] is not ready!', level='Warn')

    def _cb_menu_center(self, *args):
        d = dialog_center(self, initialvalue=self._params['laser_line_center_node'])
        if d is None:
            return
        future = self.ros.set_params('laser_line_center_node', d)
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'laser_line_center_node': d}))
        else:
            self._msg('Service [laser_line_center_node] is not ready!', level='Warn')

    def _cb_menu_line_filter(self, *args):
        d = dialog_line_filter(self, initialvalue=self._params['laser_line_filter_node'])
        if d is None:
            return
        future = self.ros.set_params('laser_line_filter_node', d)
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'laser_line_filter_node': d}))
        else:
            self._msg('Service [laser_line_filter_node] is not ready!', level='Warn')

    def _cb_menu_seam_filter(self, *args):
        d = dialog_seam_filter(self, initialvalue=self._params['seam_tracking_node'])
        if d is None:
            return
        future = self.ros.set_params('seam_tracking_node', d)
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'seam_tracking_node': d}))
        else:
            self._msg('Service [seam_tracking_node] is not ready!', level='Warn')

    def _cb_menu_preserve_config(self, *args):
        self._msg('Menu [Preserve config] clicked')
        msg = yaml.dump(self._params)
        self.ros.pub_config(msg)

    def _cb_menu_reboot_defaults(self, *args):
        self._msg('Menu [Reboot defaults] clicked')
        self.ros.pub_config('')
        self.ros.pub_config('restart')

    def _params_cb_power(self, b: bool):
        if b:
            self.btn_power['text'] = 'Camera off'
            self.btn_power.state(['pressed'])
        else:
            self.btn_power['text'] = 'Camera on'
            self.btn_power.state(['!pressed'])

    def _params_cb_laser(self, b: bool):
        if b:
            self.btn_laser['text'] = 'Laser off'
            self.btn_laser.state(['pressed'])
        else:
            self.btn_laser['text'] = 'Laser on'
            self.btn_laser.state(['!pressed'])

    def _params_cb_task(self, task: int):
        self._task = task
        self._update_codes()

    def _cb_btn_power(self, *args):
        if self.btn_power['text'] == 'Camera on':
            self._msg('Button [Camera on] clicked')
            future = self.ros.set_params('camera_tis_node', {'power': True})
            if future is not None:
                future.add_done_callback(
                    lambda f: self._cb_set_params_done(f, {'camera_tis_node': {'power': True}}))
            else:
                self._msg('Service [camera_tis_node] is not ready!', level='Warn')
        else:
            self._msg('Button [Camera off] clicked')
            future = self.ros.set_params('camera_tis_node', {'power': False})
            if future is not None:
                future.add_done_callback(
                    lambda f: self._cb_set_params_done(f, {'camera_tis_node': {'power': False}}))
            else:
                self._msg('Service [camera_tis_node] is not ready!', level='Warn')

    def _cb_btn_laser(self, *args):
        gpn = 'gpio_raspberry_node'
        if self.btn_laser['text'] == 'Laser on':
            self._msg('Button [Laser on] clicked')
            future = self.ros.set_params(gpn, {'laser': True})
            if future is not None:
                future.add_done_callback(
                    lambda f: self._cb_set_params_done(f, {gpn: {'laser': True}}))
            else:
                self._msg('Service [gpio_raspberry_node] is not ready!', level='Warn')
        else:
            self._msg('Button [Laser off] clicked')
            future = self.ros.set_params(gpn, {'laser': False})
            if future is not None:
                future.add_done_callback(
                    lambda f: self._cb_set_params_done(f, {gpn: {'laser': False}}))
            else:
                self._msg('Service [gpio_raspberry_node] is not ready!', level='Warn')

    def _cb_btn_task(self, *args):
        self._msg('Button [Task] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        v = self._params['seam_tracking_node']['task']
        v = str(v + 1) if v >= 0 else ''
        task = simpledialog.askinteger('Task', 'Input task ID:', initialvalue=v)
        if task is None:
            return
        future = self.ros.set_params('seam_tracking_node', {'task': task - 1})
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'seam_tracking_node': {'task': task - 1}}))
        else:
            self._msg('Service [seam_tracking_node] is not ready!', level='Warn')

    def _cb_btn_backup(self, *args):
        self._msg('Button [Backup] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        codes = self._params['seam_tracking_node']['codes']
        filename = filedialog.asksaveasfilename(
            title='Backup codes',
            initialfile='codes.json',
            defaultextension='json',
            filetypes=[('JSON JavaScript Object Notation', '.json'),
                       ("YAML Ain't Markup Language", '.yaml .yml')])
        if filename:
            try:
                fp = open(filename, 'w')
                if filename.endswith('.json'):
                    json.dump(codes, fp)
                elif filename.endswith(('.yaml', '.yml')):
                    yaml.dump(codes, fp)
                else:
                    raise TypeError('Only JSON or YAML files are supported')
                self._msg('Backup done!')
            except Exception as e:
                self._msg(f'{str(e)}', level='Error')
            finally:
                fp.close()

    def _cb_btn_upload(self, *args):
        self._msg('Button [Upload] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        codes = self._params['seam_tracking_node']['codes']
        filename = filedialog.askopenfilename(
            title='Upload codes',
            initialfile='codes.json',
            defaultextension='json',
            filetypes=[('JSON JavaScript Object Notation', '.json'),
                       ("YAML Ain't Markup Language", '.yaml .yml')])
        if filename:
            try:
                fp = open(filename)
                if filename.endswith('.json'):
                    codes[:] = json.load(fp)
                elif filename.endswith(('.yaml', '.yml')):
                    codes[:] = yaml.load(fp)
                else:
                    raise TypeError('Only JSON or YAML files are supported')
                self._update_codes()
                self._msg('Upload done!')
            except Exception as e:
                self._msg(f'{str(e)}', level='Error')
            finally:
                fp.close()

    def _cb_btn_refresh(self, *args):
        self._msg('Button [Refresh] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        self._refresh()

    def _cb_btn_previous(self, *args):
        self._msg('Button [Previous] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        self._task -= 1
        self._update_codes()

    def _cb_btn_next(self, *args):
        self._msg('Button [Next] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        self._task += 1
        self._update_codes()

    def _cb_btn_append(self, *args):
        self._msg('Button [Append] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        codes = self._params['seam_tracking_node']['codes']
        codes.append('def fn(d):\n    return None')
        self._task = len(codes) - 1
        self._update_codes()

    def _cb_btn_delete(self, *args):
        self._msg('Button [Delete] clicked')
        codes = self._params['seam_tracking_node']['codes']
        del codes[self._task]
        self._update_codes()

    def _cb_btn_modify(self, *args):
        self._msg('Button [Modify] clicked')
        codes = self._params['seam_tracking_node']['codes']
        codes[self._task] = self.texts.get('1.0', 'end').rstrip()

    def _cb_btn_commit(self, *args):
        self._msg('Button [Commit] clicked')
        if self._code_modified():
            yes = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if not yes:
                return
        codes = self._params['seam_tracking_node']['codes']
        future = self.ros.set_params('seam_tracking_node', {'codes': codes})
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_set_params_done(f, {'seam_tracking_node': {'codes': codes}}))
        else:
            self._msg('Service [seam_tracking_node] is not ready!', level='Warn')

    def _refresh(self):
        for n, d in self._params.items():
            f = self.ros.get_params(n, d.keys())
            if f is not None:
                f.add_done_callback(
                    lambda f, n=n, d=d: self._cb_get_params_done(f, {n: d.keys()}))
            else:
                self._msg(f'Service [{n}] is not ready!', level='Warn')

    def _update_codes(self):
        codes = self._params['seam_tracking_node']['codes']

        if not codes:
            self.btn_task['text'] = 'Task:     '
            self.btn_previous.state(['disabled'])
            self.btn_next.state(['disabled'])
            self.btn_delete.state(['disabled'])
            self.btn_modify.state(['disabled'])
            self.btn_commit.state(['disabled'])
            self.texts.replace(1.0, "end", '')
        else:
            if self._task >= len(codes):
                self._task = len(codes) - 1
            elif self._task < 0:
                self._task = 0
            if self._task == self._params['seam_tracking_node']['task']:
                self.btn_task['text'] = f'Task({len(codes)}): {self._task + 1:>2}*'
            else:
                self.btn_task['text'] = f'Task({len(codes)}): {self._task + 1:>2} '
            if self._task == 0:
                self.btn_previous.state(['disabled'])
            else:
                self.btn_previous.state(['!disabled'])
            if self._task == len(codes) - 1:
                self.btn_next.state(['disabled'])
            else:
                self.btn_next.state(['!disabled'])
            self.btn_delete.state(['!disabled'])
            self.btn_modify.state(['!disabled'])
            self.btn_commit.state(['!disabled'])
            self.texts.replace(1.0, "end", codes[self._task])

    def _code_modified(self):
        codes = self._params['seam_tracking_node']['codes']
        if not codes or codes[self._task].rstrip() == self.texts.get('1.0', 'end').rstrip():
            return False
        else:
            return True

    def _msg(self, s: str, *, level='Info'):
        with self._lock:
            n = datetime.now()
            t = n.strftime("%m/%d/%Y %H:%M:%S")
            s = f'[{t}] [{level:^10}]  {s}\n'
            self.status.insert('1.0', s)


if __name__ == '__main__':
    rclpy.init()

    app = App()

    app.mainloop()

    rclpy.shutdown()
