from src.ros_bridge.subscribers.axes_subscriber import AxesSubscriber
from src.ros_bridge.subscribers.image_subscriber import ImageSubscriber


# singleton for all subscribers
# noinspection PyClassHasNoInit
class Singleton:
    __instance = None

    @staticmethod
    def get_instance():
        """ Static access method. """
        if Singleton.__instance is None:
            Singleton()
        return Singleton.__instance

    def __init__(self):
        """ Virtually private constructor. """
        if Singleton.__instance is not None:
            raise Exception("This class is a singleton!")
        else:
            Singleton.__instance = self
            self.axes_subscriber = AxesSubscriber()
            self.image_subscriber = ImageSubscriber()
