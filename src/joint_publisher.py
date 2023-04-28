#!/usr/bin/env python

import rospy
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface

class JointPublisher:
    def __init__(self):
        rospy.init_node("joint_publisher", anonymous=False)

        # Connects to the robot
        self.rtde_c = RTDEControlInterface("0.0.0.0")
        self.rtde_r = RTDEReceiveInterface("0.0.0.0")

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    joint_publisher = JointPublisher()
    joint_publisher.main()