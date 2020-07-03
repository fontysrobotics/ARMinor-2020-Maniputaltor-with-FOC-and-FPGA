import cv2
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage


class ImageSubscriber:
    cv_image = np.zeros((480, 640, 3), np.uint8)
    last_msg_seq = -1

    def __init__(self):
        self.bridge = CvBridge()
        self.thread = threading.Thread(target=self.image_listener)
        self.thread.start()

    def msg_cb(self, data):
        self.last_msg_seq = data.header.seq
        try:
            self.cv_image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(data, "passthrough"), cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            print e

    def image_listener(self):
        rospy.Subscriber("/camera/rgb/image_raw", CompressedImage, self.msg_cb)
        rospy.spin()


if __name__ == '__main__':
    ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shut down")
    cv2.destroyAllWindows()
