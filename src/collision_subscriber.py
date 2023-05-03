#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from rtde_control import RTDEControlInterface
from rtde_io import RTDEIOInterface

class CollisionSubscriber:
    def __init__(self):
        rospy.init_node("collision_subscriber", anonymous=False)
        rospy.Subscriber("/collision", Bool, self.clbk_collision)

        self.rtde_c = RTDEControlInterface("192.168.50.85")
        self.rtde_io = RTDEIOInterface("192.168.50.85")
    

    def clbk_collision(self, msg):
        if msg.data:
            # Stop the robot in force mode
            self.rtde_c.force_mode(pose=[0,0,0,0,0,0], force=[0,0,0,0,0,0], selection_vector=[1,1,1,0,0,0], target_velocity=[0,0,0,0,0,0], force_type=2)
            self.rtde_io.set_standard_digital_out(0, True)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    collision_subscriber = CollisionSubscriber()
    collision_subscriber.main()