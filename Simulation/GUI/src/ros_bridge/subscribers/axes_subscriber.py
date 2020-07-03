import threading

import rospy
from sensor_msgs.msg import JointState


class AxesSubscriber:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_msg_seq = -1
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

    def joint_states_listener(self):
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        rospy.spin()

    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.last_msg_seq = msg.header.seq
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.lock.release()

    def return_joint_state(self, joint_name):
        # todo position is wrong
        if not self.name:
            return 0, 0., 0., 0.

        self.lock.acquire()
        if joint_name in self.name:
            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]
            self.lock.release()
            return index, position, velocity, effort

        rospy.logerr("Joint {} not found".format(joint_name))
        self.lock.release()
        return 0, 0., 0., 0.
