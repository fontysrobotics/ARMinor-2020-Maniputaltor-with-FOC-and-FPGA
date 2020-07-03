import Tkinter as Tk

from src.ros_bridge.publishers.camera_control_publisher import CameraControlPublisher


class CameraControlFrame(Tk.Frame):
    def __init__(self, parent, **kwargs):
        self._pub = CameraControlPublisher()
        Tk.Frame.__init__(self, parent, **kwargs)
        self._label = Tk.Label(self, text='Camera control')
        self._label.pack(side=Tk.TOP, padx=(8, 8))

        self._down_btn = Tk.Button(self, command=self._down_btn_action, text="Rotate down")
        self._down_btn.pack(side=Tk.LEFT)

        self._left_btn = Tk.Button(self, command=self._left_btn_action, text="Rotate Left")
        self._left_btn.pack(side=Tk.LEFT)

        self._backward_btn = Tk.Button(self, command=self._backward_btn_action, text="Move backward")
        self._backward_btn.pack(side=Tk.LEFT)

        self._forward_btn = Tk.Button(self, command=self._forward_btn_action, text="Move forward")
        self._forward_btn.pack(side=Tk.LEFT)

        self._right_btn = Tk.Button(self, command=self._right_btn_action, text="Rotate Right")
        self._right_btn.pack(side=Tk.LEFT)

        self._ccw_btn = Tk.Button(self, command=self._ccw_btn_action, text="Rotate up")
        self._ccw_btn.pack(side=Tk.LEFT)

    def _down_btn_action(self):
        self._pub.publish_message(z=-0.3)

    def _forward_btn_action(self):
        self._pub.publish_message(x=1)

    def _ccw_btn_action(self):
        self._pub.publish_message(z=0.3)

    def _left_btn_action(self):
        self._pub.publish_message(y=0.3)

    def _right_btn_action(self):
        self._pub.publish_message(y=-0.3)

    def _backward_btn_action(self):
        self._pub.publish_message(x=-1)
