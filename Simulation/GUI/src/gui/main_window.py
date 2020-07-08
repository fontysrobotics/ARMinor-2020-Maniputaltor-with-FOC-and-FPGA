#!/usr/bin/env python2

import Tkinter as Tk

import rospy

from src.gui.move.move_frame import MoveFrame
from src.gui.visualisation.robot_frame import RobotFrame

form = Tk.Tk()


def on_close():
    global form
    rospy.signal_shutdown("User closed the window")
    form.destroy()


def start_gui():
    global form
    rospy.init_node("manarm_gui")
    form.title("ManArm controller")

    title_label = Tk.Label(form, text="ManArm controller")
    title_label.pack()

    move_frame = MoveFrame(form)
    move_frame.pack(side=Tk.LEFT)

    robot_frame = RobotFrame(form)
    robot_frame.pack(side=Tk.LEFT)

    form.protocol("WM_DELETE_WINDOW", on_close)
    form.mainloop()


if __name__ == '__main__':
    start_gui()
