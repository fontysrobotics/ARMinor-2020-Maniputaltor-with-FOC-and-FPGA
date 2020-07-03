import rospy
from geometry_msgs.msg import Twist


class CameraControlPublisher:

    def __init__(self):
        self._pub = rospy.Publisher('/camera/control', Twist, queue_size=10)

    def publish_message(self, x=0, y=0, z=0):
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        self._pub.publish(msg)
