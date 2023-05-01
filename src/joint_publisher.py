#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface

class JointPublisher:
    def __init__(self):
        rospy.init_node("joint_publisher", anonymous=False)

        # Creates a publisher for the "/joints" ROS topic
        self.joint_pub = rospy.Publisher("/joints", JointState, queue_size=10)

        # Connects to the robot
        self.rtde_c = RTDEControlInterface("0.0.0.0")
        self.rtde_r = RTDEReceiveInterface("0.0.0.0")

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Gets actual joint data
            joint_positions = self.rtde_r.getActualQ()
            joint_velocities = self.rtde_r.getActualQd()

            # Creates the JointState message
            joint_msg = JointState()
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.name = ['shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
            joint_msg.position = joint_positions
            joint_msg.velocity = joint_velocities

            # Publishes the data to the ROS topic
            self.joint_pub.publish(joints)
            
            r.sleep()

if __name__ == '__main__':
    joint_publisher = JointPublisher()
    joint_publisher.main()