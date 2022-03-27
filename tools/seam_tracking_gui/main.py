import rclpy
import tkinter as tk
from threading import Thread

from tkinter import ttk, simpledialog, messagebox, filedialog
from tkinter.scrolledtext import ScrolledText
from ros_node import RosNode
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from point_data import PointData
from custom_figure import CustomFigure
from custom_dialog import mydialog
from codes import Codes
from datetime import datetime

class App(tk.Tk):
    """Toplevel window."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._params = {
            'exposure_time': None,
            'power': False,
            'laser': False,
            'task': None,
            'delta_x': None,
            'delta_y': None
        }

        self.title('Seam Tracking GUI')
        self.option_add('*tearOff', False)
        self.protocol("WM_DELETE_WINDOW", self.__exit)

        self.pnts_data = PointData()
        self.seam_data = PointData()
        self.codes = Codes()

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

        self.ros = RosNode()
        self.ros.sub_pnts(self._ros_cb_pnts)
        self.ros.sub_seam(self._ros_cb_seam)
        self.ros.sub_log(self._ros_cb_log)

        self._thread = Thread(target=rclpy.spin, args=[self.ros])
        self._thread.start()

        self._refresh()

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

        self.bind('<<RosSubPnts>>', lambda e: fig.update_pnts(self.pnts_data))
        self.bind('<<RosSubPnts>>', lambda e: canvas.draw_idle(), add='+')
        self.bind('<<RosSubSeam>>', lambda e: fig.update_seam(self.seam_data))
        self.bind('<<RosSubSeam>>', lambda e: canvas.draw_idle(), add='+')
        return frame

    def _init_list(self):
        frame = ttk.Frame(self, padding=(10,0,10,0))

        self.btn_task = ttk.Button(frame, text='Task:', width=10, command=self._cb_btn_task)
        self.btn_previous = ttk.Button(frame, text='Previous', width=10, command=self._cb_btn_previous)
        self.btn_next = ttk.Button(frame, text='Next', width=10, command=self._cb_btn_next)
        self.btn_refresh = ttk.Button(frame, text='Refresh', width=10, command=self._cb_btn_refresh)

        self.texts = ScrolledText(frame, wrap = 'none')

        self.btn_laser = ttk.Button(frame, text='Laser on', width=10, command=self._cb_btn_laser)
        self.btn_power = ttk.Button(frame, text='Camera on', width=10, command=self._cb_btn_power)

        self.btn_append = ttk.Button(frame, text='Append', width=10, command=self._cb_btn_append)
        self.btn_delete = ttk.Button(frame, text='Delete', width=10, command=self._cb_btn_delete)
        self.btn_modify = ttk.Button(frame, text='Modify', width=10, command=self._cb_btn_modify)
    
        self.btn_commit = ttk.Button(frame, text='Commit', width=10, command=self._cb_btn_commit)

        self.btn_backup = ttk.Button(frame, text='Backup...', width=10, command=self._cb_btn_backup)
        self.btn_upload = ttk.Button(frame, text='Upload...', width=10, command=self._cb_btn_upload)

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

        self.bind('<<UpdateCode>>', lambda e: self.texts.replace(1.0, "end", self.codes.code()))
        return frame

    def _init_menu(self):
        menubar = tk.Menu(self)
        self['menu'] = menubar

        menu_file = tk.Menu(menubar)
        menu_file.add_command(label='New')
        menu_file.add_command(label='Open...')
        menu_file.add_command(label='Close')

        menu_edit = tk.Menu(menubar)
        menu_edit.add_command(label='Exposure time...', command=self._cb_menu_exposure)
        menu_edit.add_command(label='Offset...', command=self._cb_menu_offset)
        menu_help = tk.Menu(menubar)

        menubar.add_cascade(menu=menu_file, label='File')
        menubar.add_cascade(menu=menu_edit, label='Edit')
        menubar.add_cascade(menu=menu_help, label='Help')

    def _ros_cb_pnts(self, msg):
        self.pnts_data.from_msg(msg)
        self.event_generate('<<RosSubPnts>>', when='tail')

    def _ros_cb_seam(self, msg):
        self.seam_data.from_msg(msg)
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

    def _cb_menu_exposure_done(self, future, exposure):
        try:
            res, = future.result().results
            if res.successful:
                self._params['exposure_time'] = exposure
                self._msg(f'Exposure time set to: {exposure} (us)')
            else:
                self._msg(f'{res.reason}', level='Warn')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _cb_menu_exposure(self, *args):
        v = self._params['exposure_time']
        v = str(v) if v is not None else ''
        exposure = simpledialog.askinteger('Exposure time', 'Input exposure time:', initialvalue=v)
        if exposure is None:
            return
        future = self.ros.camera_set({'exposure_time': exposure})
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_menu_exposure_done(f, exposure)
            )
        else:
            self._msg('Service exposure_time is not ready!', level='Warn')

    def _cb_menu_offset_done(self, future, dx, dy):
        try:
            rx, ry = future.result().results
            if rx.successful:
                self._params['delta_x'] = dx
                self._msg(f'Offset x set to: {dx:.2f} (mm)')
            else:
                self._msg(f'{rx.reason}', level='Warn')
            if ry.successful:
                self._params['delta_y'] = dy
                self._msg(f'Offset y set to: {dy:.2f} (mm)')
            else:
                self._msg(f'{ry.reason}', level='Warn')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _cb_menu_offset(self, *args):
        x, y = self._params['delta_x'], self._params['delta_y']
        x = f'{x:.2f}' if x is not None else ''
        y = f'{y:.2f}' if y is not None else ''

        dx, dy = mydialog(self, initialvalue=(x, y))
        if dx is None or dy is None:
            return
        future = self.ros.seam_set({
            'delta_x': dx,
            'delta_y': dy
        })
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_menu_offset_done(f, dx, dy)
            )
        else:
            self._msg('Service delta is not ready!', level='Warn')

    def _cb_btn_laser(self, *args):
        if self.btn_laser['text'] == 'Laser on':
            self._msg('Button [Laser on] clicked')
            future = self.ros.gpio_set({'laser': True})
            if future is not None:
                self.btn_laser.state(['pressed'])
                future.add_done_callback(self._cb_btn_laser_on_done)
            else:
                self._msg('Service gpio set parameters is not ready!', level='Warn')
        else:
            self._msg('Button [Laser off] clicked')
            future = self.ros.gpio_set({'laser': False})
            if future is not None:
                self.btn_laser.state(['!pressed'])
                future.add_done_callback(self._cb_btn_laser_off_done)
            else:
                self._msg('Service gpio set parameters is not ready!', level='Warn')
    
    def _cb_btn_laser_on_done(self, future):
        try:
            res, = future.result().results
            if res.successful:
                self._params['laser'] = True
                self.btn_laser['text'] = 'Laser off'
                self._msg(f'Laser set to: on')
            else:
                self._msg(f'{res.reason}', level='Warn')
                self.btn_laser.state(['!pressed'])
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')
            self.btn_laser.state(['!pressed'])

    def _cb_btn_laser_off_done(self, future):
        try:
            res, = future.result().results
            if res.successful:
                self._params['laser'] = False
                self.btn_laser['text'] = 'Laser on'
                self._msg(f'Laser set to: off')
            else:
                self._msg(f'{res.reason}', level='Warn')
                self.btn_laser.state(['pressed'])
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')
            self.btn_laser.state(['pressed'])

    def _cb_btn_power(self, *args):
        if self.btn_power['text'] == 'Camera on':
            self._msg('Button [Camera on] clicked')
            future = self.ros.camera_set({'power': True})
            if future is not None:
                self.btn_power.state(['pressed'])
                future.add_done_callback(self._cb_btn_power_on_done)
            else:
                self._msg('Service camera set parameters is not ready!', level='Warn')
        else:
            self._msg('Button [Camera off] clicked')
            future = self.ros.camera_set({'power': False})
            if future is not None:
                self.btn_power.state(['!pressed'])
                future.add_done_callback(self._cb_btn_power_off_done)
            else:
                self._msg('Service camera set parameters is not ready!', level='Warn')
    
    def _cb_btn_power_on_done(self, future):
        try:
            res, = future.result().results
            if res.successful:
                self._params['power'] = True
                self.btn_power['text'] = 'Camera off'
                self._msg(f'Camera set to: on')
            else:
                self._msg(f'{res.reason}', level='Warn')
                self.btn_power.state(['!pressed'])
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')
            self.btn_power.state(['!pressed'])

    def _cb_btn_power_off_done(self, future):
        try:
            res, = future.result().results
            if res.successful:
                self._params['power'] = False
                self.btn_power['text'] = 'Camera on'
                self._msg(f'Camera set to: off')
            else:
                self._msg(f'{res.reason}', level='Warn')
                self.btn_power.state(['pressed'])
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')
            self.btn_power.state(['pressed'])

    def _cb_btn_task_done(self, future, task):
        try:
            res, = future.result().results
            if res.successful:
                self._params['task'] = task
                self.codes.goto(id=task)
                self._update_codes()
                self._msg(f'Task set to: {task}')
            else:
                self._msg(f'{res.reason}', level='Warn')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _cb_btn_task(self, *args):
        self._msg('Button [Task] clicked')
        v = self._params['task']
        v = str(v) if v is not None else ''
        task = simpledialog.askinteger('Task', 'Input task ID:', initialvalue=v)
        if task is None:
            return
        future = self.ros.seam_set({'task': task})
        if future is not None:
            future.add_done_callback(
                lambda f: self._cb_btn_task_done(f, task)
            )
        else:
            self._msg('Service seam set parameters is not ready!', level='Warn')

    def _cb_btn_backup(self, *args):
        self._msg('Button [Backup] clicked')
        filename = filedialog.asksaveasfilename(
            title='Backup codes',
            initialfile='codes.json',
            defaultextension='json',
            filetypes=[('JSON JavaScript Object Notation', '.json')])
        if filename:
            try:
                self.codes.dump(filename)
                self._msg('Backup done!')
            except Exception as e:
                self._msg(f'{str(e)}', level='Error')

    def _cb_btn_upload(self, *args):
        self._msg('Button [Upload] clicked')
        filename = filedialog.askopenfilename(
            title='Upload codes',
            initialfile='codes.json',
            defaultextension='json',
            filetypes=[('JSON JavaScript Object Notation', '.json')])
        if filename:
            try:
                self.codes.load(filename)
                self._update_codes()
                self._msg('Upload done!')
            except Exception as e:
                self._msg(f'{str(e)}', level='Error')

    def _get_codes_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.codes.loads(res.codes)
                self.codes.goto(id=self._params['task'])
                self._update_codes()
            else:
                self._msg(f'{res.message}', level='Warn')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _camera_get_done(self, future):
        try:
            power, exposure = future.result().values
            self._params['exposure_time'] = exposure.integer_value
            if power.bool_value:
                self._params['power'] = True
                self.btn_power['text'] = 'Camera off'
                self.btn_power.state(['pressed'])
            else:
                self._params['power'] = False
                self.btn_power['text'] = 'Camera on'
                self.btn_power.state(['!pressed'])
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _gpio_get_done(self, future):
        try:
            laser, = future.result().values
            if laser.bool_value:
                self._params['laser'] = True
                self.btn_laser['text'] = 'Laser off'
                self.btn_laser.state(['pressed'])
            else:
                self._params['laser'] = False
                self.btn_laser['text'] = 'Laser on'
                self.btn_laser.state(['!pressed'])
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _seam_get_done(self, future):
        try:
            task, delta_x, delta_y = future.result().values
            task = task.integer_value
            self._params['task'] = task
            self._params['delta_x'] = delta_x.double_value
            self._params['delta_y'] = delta_y.double_value
            self.codes.goto(id=task)
            self._update_codes()
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _refresh(self):
        if f := self.ros.get_codes():
            f.add_done_callback(self._get_codes_done)
        else:
            self._msg('Service seam get codes is not ready!', level='Warn')

        if f := self.ros.camera_get(['power', 'exposure_time']):
            f.add_done_callback(self._camera_get_done)
        else:
            self._msg('Service camera get parameters is not ready!', level='Warn')

        if f := self.ros.gpio_get(['laser']):
            f.add_done_callback(self._gpio_get_done)
        else:
            self._msg('Service gpio get parameters is not ready!', level='Warn')

        if f := self.ros.seam_get(['task', 'delta_x', 'delta_y']):
            f.add_done_callback(self._seam_get_done)
        else:
            self._msg('Service seam get parameters is not ready!', level='Warn')

    def _cb_btn_refresh(self, *args):
        self._msg('Button [Refresh] clicked')
        if self._code_modified():
            answer = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if answer:
                self._refresh()
        else:
            self._refresh()

    def _cb_btn_previous(self, *args):
        self._msg('Button [Previous] clicked')
        if self._code_modified():
            answer = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if answer:
                self.codes.previous()
                self._update_codes()
        else:
            self.codes.previous()
            self._update_codes()

    def _cb_btn_next(self, *args):
        self._msg('Button [Next] clicked')
        if self._code_modified():
            answer = messagebox.askyesno('Question', message='Code modified, leave anyway?')
            if answer:
                self.codes.next()
                self._update_codes()
        else:
            self.codes.next()
            self._update_codes()

    def _cb_btn_append(self, *args):
        self._msg('Button [Append] clicked')
        self.codes.append_code('def fn(x, y, *args):\n    return [], []')
        self._update_codes()

    def _cb_btn_delete(self, *args):
        self._msg('Button [Delete] clicked')
        self.codes.delete_code()
        self._update_codes()

    def _cb_btn_modify(self, *args):
        self._msg('Button [Modify] clicked')
        self.codes.modify_code(self.texts.get('1.0', 'end').rstrip())
        self._update_codes()

    def _cb_btn_commit(self, *args):
        self._msg('Button [Commit] clicked')
        s = self.codes.dumps()
        future = self.ros.set_codes(s)
        if future is not None:
            future.add_done_callback(self._cb_btn_commit_done)
        else:
            self._msg('Service seam set codes is not ready!', level='Warn')

    def _cb_btn_commit_done(self, future):
        try:
            res = future.result()
            if res.success:
                self._msg('Commit done!')
            else:
                self._msg(f'{res.message}', level='Warn')
        except Exception as e:
            self._msg(f'{str(e)}', level='Error')

    def _update_codes(self):
        pos = self.codes.pos()
        if pos is None:
            self.btn_task['text'] = 'Task:     '
        else:
            if pos == self._params['task']:
                self.btn_task['text'] = f'Task: {pos:>2}*'
            else:
                self.btn_task['text'] = f'Task: {pos:>2} '

        if self.codes.is_valid():
            self.btn_delete.state(['!disabled'])
            self.btn_modify.state(['!disabled'])
            self.btn_commit.state(['!disabled'])
        else:
            self.btn_delete.state(['disabled'])
            self.btn_modify.state(['disabled'])
            self.btn_commit.state(['disabled'])

        if self.codes.is_begin():
            self.btn_previous.state(['disabled'])
        else:
            self.btn_previous.state(['!disabled'])

        if self.codes.is_end():
            self.btn_next.state(['disabled'])
        else:
            self.btn_next.state(['!disabled'])

        self.event_generate('<<UpdateCode>>', when='tail')

    def _code_modified(self):
        if self.codes.code().rstrip() == self.texts.get('1.0', 'end').rstrip():
            return False
        else:
            return True

    def _msg(self, s: str, *, level = 'Info'):
        n = datetime.now()
        t = n.strftime("%m/%d/%Y %H:%M:%S")
        s = f'[{t}] [{level:^10}]  {s}\n'
        self.status.insert('1.0', s)

if __name__ == '__main__':
    rclpy.init()

    app = App()

    app.mainloop()

    rclpy.shutdown()
