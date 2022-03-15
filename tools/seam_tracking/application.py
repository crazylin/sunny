import tkinter as tk
import rclpy
import seam_tracking as st

from custom_figure import CustomFigure
from tkinter import ttk
from ros_thread import RosThread

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg, NavigationToolbar2Tk)

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from shared_interfaces.msg import ModbusCoord
from shared_interfaces.srv import GetCode
from std_srvs.srv import Trigger
import ros2_numpy as rnp


class App(tk.Tk):
    """Toplevel window."""

    def __init__(self):
        super().__init__()

        self.line_data = st.LineData()
        self.pick_data = st.PickData()
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
    
    def __exit(self):
        while rclpy.ok():
            rclpy.try_shutdown()
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
        btnAdd = ttk.Button(frame, text='Add', width=10)
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

def callbackLine(msg, line_data, root):
    header, data = st.msg2dict(msg)
    # print(cy, cz)
    # if py != None:
    #     pick = st.xyz2np(0, py, pz)
    #     seam_data.write(info, data, pick = pick)
    # else:
    line_data.write(header, [data['y'], data['z']])
    root.event_generate('<<RosSubLine>>', when='tail')

def callbackPick(msg, pick_data, root):
    if msg.valid:
        pick_data.write([[msg.y], [msg.z]])
    else:
        pick_data.write([[], []])
    root.event_generate('<<RosSubPick>>', when='tail')

if __name__ == '__main__':
    
    app = App()


    ros = RosThread('seam_tracking_gui')

    qos = qos_profile_sensor_data
    qos.depth = 1
    ros.create_subscription('line',
                            PointCloud2,
                            '/line_center_reconstruction_node/pnts',
                            lambda msg: callbackLine(msg, app.line_data, app),
                            qos)
    ros.create_subscription('pick',
                            ModbusCoord,
                            '/seam_tracking_node/coord',
                            lambda msg: callbackPick(msg, app.pick_data, app),
                            10)
    # ros.create_client('get',
    #                   GetCode,
    #                   '/seam_tracking_node/get_code')
    # ros.create_client('set',
    #                   )
    # ros.create_client('gpio_high',
    #                   Trigger,
    #                   '/gpio')
    
    # ros.create_client('gpio_low',
    #                   Trigger,
    #                   'gpio_low')
    ros.start()          
    app.mainloop()
    ros.join()
