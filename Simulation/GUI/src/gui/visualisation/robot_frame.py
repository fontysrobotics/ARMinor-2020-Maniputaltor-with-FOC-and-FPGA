import Tkinter as Tk

from src.gui.visualisation.camera_control import CameraControlFrame
from src.gui.visualisation.robot_image import CameraImageFrame


class RobotFrame(Tk.Frame):
    def __init__(self, parent, **kwargs):
        Tk.Frame.__init__(self, parent, **kwargs)
        self._camera_frame = CameraImageFrame(self)
        self._camera_frame.pack(side=Tk.TOP)
        self._control_frame = CameraControlFrame(self)
        self._control_frame.pack(side=Tk.TOP)
