#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface

class JointPublisher:
    def __init__(self):
        rospy.init_node("joint_publisher", anonymous=False)

        # Connects to the robot
        self.rtde_c = RTDEControlInterface("0.0.0.0")
        self.rtde_r = RTDEReceiveInterface("0.0.0.0")
        # Creates a publisher for the "/joints" ROS topic
        self.joint_pub = rospy.Publisher("/joints", JointState, queue_size=10)

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Gets actual joint data
            joints = self.rtde_r.getActualQ()
            # Publishes the data to the ROS topic
            self.joint_pub.publish(joints)
            r.sleep()

if __name__ == '__main__':
    joint_publisher = JointPublisher()
    joint_publisher.main()