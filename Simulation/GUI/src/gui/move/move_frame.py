import Tkinter as Tk

from axes_panel import AxesPanel


# todo add linear end effector movement controls
class MoveFrame(Tk.Frame):
    """ frame that holds all widgets for controlling the arm """

    def __init__(self, parent, **kw):
        Tk.Frame.__init__(self, parent, **kw)
        self._axes_panel = AxesPanel(self)
        self._axes_panel.pack()
