#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from ur_rtde.rtde import RTDEControlInterface, RTDEIOInterface

def callback(data):
    if data.data:
        # Stop the robot in force mode
        rtde_c = RTDEControlInterface("192.168.50.85")
        rtde_io = RTDEIOInterface("192.168.50.85")
        rtde_c.force_mode(pose=[0,0,0,0,0,0], force=[0,0,0,0,0,0], selection_vector=[1,1,1,0,0,0], target_velocity=[0,0,0,0,0,0], force_type=2)
        rtde_io.set_standard_digital_out(0, True)

def ur_rtde_node():
    rospy.init_node('ur_rtde_node')
    rospy.Subscriber('collision_detected', Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        ur_rtde_node()
    except rospy.ROSInterruptException:
        pass
