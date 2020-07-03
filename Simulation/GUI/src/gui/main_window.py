import Tkinter as Tk

import rospy

from src.gui.move.move_frame import MoveFrame
from src.gui.visualisation.robot_frame import RobotFrame

# todo the window doesn't stop properly

rospy.init_node("manarm_gui")

form = Tk.Tk()
form.title("ManArm controller")

myLabel = Tk.Label(form, text="ManArm controller")
myLabel.pack()

move_frame = MoveFrame(form)
move_frame.pack(side=Tk.LEFT)

robot_frame = RobotFrame(form)
robot_frame.pack(side=Tk.LEFT)

form.mainloop()
