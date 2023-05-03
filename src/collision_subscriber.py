#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from rtde_control import RTDEControlInterface
from rtde_io import RTDEIOInterface

class CollisionSubscriber:
    def __init__(self):
        rospy.init_node("collision_subscriber", anonymous=False)
        
        self.collisions = 0
        self.force_mode_enabled = False
        
        # Creates a subscriber for the "/collision" ROS topic
        self.collision_sub = rospy.Subscriber("/collision", Bool, self.clbk_collision)

        # Connects to the robot
        self.rtde_c = RTDEControlInterface("192.168.50.85")
        self.rtde_io = RTDEIOInterface("192.168.50.85")
    

    def clbk_collision(self, msg):
        # ROS callbacks are threadsafe by default
        
        # Checks if the callback is a collision (true) or a separation (false)
        if msg.data:
            self.collisions += 1
            
            if self.force_mode_enabled:
                # Enables force mode
                self.rtde_c.force_mode(pose=[0,0,0,0,0,0], force=[0,0,0,0,0,0], selection_vector=[1,1,1,0,0,0], target_velocity=[0,0,0,0,0,0], force_type=2)
                self.rtde_io.set_standard_digital_out(0, True)

                self.force_mode_enabled = True
        else:
            self.collisions -= 1

            if self.collisions == 0:
                # TODO: Disable force mode
                self.force_mode_enabled = False

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    collision_subscriber = CollisionSubscriber()
    collision_subscriber.main()