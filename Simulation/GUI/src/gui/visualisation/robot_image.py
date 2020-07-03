import Tkinter as Tk

from PIL import Image, ImageTk

from src.ros_bridge.subscribers.topic_subscribers import Singleton


class CameraImageFrame(Tk.Frame):
    _last_img_seq = -1
    _image_id = None
    _img = None
    _update_interval = 100

    def __init__(self, parent, **kwargs):
        Tk.Frame.__init__(self, parent, kwargs)
        self._img_sub = Singleton.get_instance().image_subscriber
        h, w, n_chan = self._img_sub.cv_image.shape
        self._img_container = Tk.Canvas(parent, width=w, height=h)
        self._img_container.pack()
        self.after(self._update_interval, self.update_img)

    def update_img(self):
        print "image cb"
        if self._last_img_seq == self._img_sub.last_msg_seq:
            print "no img msg received"
            self.after(self._update_interval, self.update_img)
            return

        self._last_img_seq = self._img_sub.last_msg_seq
        self._img = ImageTk.PhotoImage(image=Image.fromarray(self._img_sub.cv_image))
        self._image_id = self._img_container.create_image(0, 0, anchor='nw', image=self._img)
        self.after(self._update_interval, self.update_img)
