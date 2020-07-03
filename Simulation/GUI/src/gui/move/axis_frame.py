import Tkinter as Tk


class AxisFrame(Tk.Frame):
    """ A frame that has an indicator and control buttons for a single axis """

    def __init__(self, parent, left_btn_action=None, right_btn_action=None, btn_released_action=None,
                 min_angle=-360, max_angle=360, axis_name="Axis", **kwargs):
        """
         #:param left_btn_action: a callback that is called when the left button is pressed
         #:param right_btn_action: a callback that is called when the right button is pressed
         #:param btn_released_action: a callback that is called when any button is released
         #:keyword
         """
        self._angle = 0
        self._min_angle = min_angle
        self._max_angle = max_angle

        Tk.Frame.__init__(self, parent, **kwargs)
        # todo add callbacks to button presses as kwargs

        self._label = Tk.Label(self, text=axis_name)
        self._label.pack(side=Tk.LEFT, padx=(8, 8))

        self._left_btn = Tk.Button(self)
        self._left_btn.bind('<Button>', left_btn_action)
        self._left_btn.bind('<ButtonRelease>', btn_released_action)
        _left_btn_img = Tk.PhotoImage(file="move/img/left-arrow-icon.png")
        self._left_btn.config(image=_left_btn_img)
        self._left_btn.image = _left_btn_img
        self._left_btn.pack(side=Tk.LEFT)

        self._angle_slider_indicator = Tk.Scale(self, from_=self._min_angle, to=self._max_angle, orient=Tk.HORIZONTAL)
        self._angle_slider_indicator.config(state=Tk.DISABLED, takefocus=0)
        self._angle_slider_indicator.pack(side=Tk.LEFT)

        self._right_btn = Tk.Button(self)
        self._right_btn.bind('<Button>', right_btn_action)
        self._right_btn.bind('<ButtonRelease>', btn_released_action)
        _right_btn_img = Tk.PhotoImage(file="move/img/right-arrow-icon.png")
        self._right_btn.config(image=_right_btn_img)
        self._right_btn.image = _right_btn_img
        self._right_btn.pack(side=Tk.LEFT)

    def set_angle(self, angle):
        if not self._min_angle <= angle <= self._max_angle:
            raise ValueError("Given angle {}".format(angle) +
                             " is not between {} and {}!".format(self._min_angle, self._max_angle))

        self._angle = angle
        self._angle_slider_indicator.config(state=Tk.NORMAL)
        self._angle_slider_indicator.set(angle)
        self._angle_slider_indicator.config(state=Tk.DISABLED)

    def get_angle(self):
        return self._angle_slider_indicator.get()
