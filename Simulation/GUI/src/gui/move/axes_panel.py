import Tkinter as Tk
from functools import partial
from math import degrees

from axis_frame import AxisFrame
from src.ros_bridge.publishers.axis_joy_publisher import AxisJoyPublisher
from src.ros_bridge.subscribers.topic_subscribers import Singleton


class AxesPanel(Tk.Frame):
    """ A frame that has indicators and controls for each axis """
    _last_msg_seq = -1
    _update_interval = 250  # how often the UI updates

    def __init__(self, parent, nr_axes=6, **kw):
        Tk.Frame.__init__(self, parent, **kw)
        self._axes_subscriber = Singleton.get_instance().axes_subscriber
        self._vel_pub = AxisJoyPublisher()
        self._axes = []
        self._nr_axes = nr_axes

        for i in range(self._nr_axes):
            l_btn_func = partial(self.send_vel, i, 1)  # function for left btn, signals axis i to move with vel 1.0
            r_btn_func = partial(self.send_vel, i, -1)
            btn_release_func = partial(self.send_vel, i, 0)  # function to stop movement when btn is released

            self._axes.append(AxisFrame(self,
                                        left_btn_action=l_btn_func,
                                        right_btn_action=r_btn_func,
                                        btn_released_action=btn_release_func,
                                        axis_name=self.get_joint_name(i).replace('_', ' ')[:-5]))  # make name prettier
            self._axes[i].pack()

        self.after(self._update_interval, self.update_axis_values)

    def update_axis_values(self):
        """ callback to update the values of the axes indicators """
        if self._last_msg_seq == self._axes_subscriber.last_msg_seq:
            self.after(self._update_interval, self.update_axis_values)
            return
        self._last_msg_seq = self._axes_subscriber.last_msg_seq

        for i in range(len(self._axes)):
            _, pos, vel, eff = self._axes_subscriber.return_joint_state(self.get_joint_name(i))
            self._axes[i].set_angle(degrees(pos))
        self.after(self._update_interval, self.update_axis_values)

    @staticmethod
    def get_joint_name(index):
        joint_names = {
            0: 'shoulder_pan_joint',
            1: 'shoulder_lift_joint',
            2: 'elbow_joint',
            3: 'wrist_1_joint',
            4: 'wrist_2_joint',
            5: 'wrist_3_joint'
        }
        return joint_names.get(index, "Unrecognised index")

    def send_vel(self, index, vel, event=None):  # pressing the button sends an event to the callback, thus 4th arg
        """ sends a velocity for only 1 axis, keeps rest 0"""

        velocities = [0.0] * self._nr_axes
        velocities[index] = vel
        self._vel_pub.publish_message(velocities)
