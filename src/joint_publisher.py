#!/usr/bin/env python

import rospy
from ur_rtde.rtde import RTDEIOInterface, RTDEControlInterface

class JointPublisher:
    def __init__(self):
        rospy.init_node("joint_publisher", anonymous=False)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    joint_publisher = JointPublisher()
    joint_publisher.main()