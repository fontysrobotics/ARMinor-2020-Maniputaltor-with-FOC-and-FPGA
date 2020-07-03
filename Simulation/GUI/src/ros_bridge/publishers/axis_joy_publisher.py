import rospy
from sensor_msgs.msg import Joy


class AxisJoyPublisher:
    def __init__(self):
        self._pub = rospy.Publisher("/manarm/joy", Joy, queue_size=10)

    def publish_message(self, velocities):
        msg = Joy()
        msg.axes = velocities
        self._pub.publish(msg)
