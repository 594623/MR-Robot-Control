#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from rtde_control import RTDEControlInterface
from rtde_io import RTDEIOInterface

class CollisionSubscriber:
    def __init__(self):
        rospy.init_node("collision_subscriber", anonymous=False)

        self.ip = rospy.get_param("/robot_ip")
        
        self.collisions = 0
        self.force_mode_enabled = False

        self.task_frame = [0, 0, 0, 0, 0, 0]
        self.selection_vector = [1, 1, 1, 1, 1, 1]
        self.wrench = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        self.force_type = 2
        self.limits = [10, 10, 10, 10, 10, 10]
        self.damping = 0.0005
        
        # Creates a subscriber for the "/collision" ROS topic
        self.collision_sub = rospy.Subscriber("/collision", Bool, self.clbk_collision)

        # Connects to the robot
        self.rtde_c = RTDEControlInterface(self.ip)
        self.rtde_io = RTDEIOInterface(self.ip)

        self.rtde_c.teachMode()
    

    def clbk_collision(self, msg):
        # ROS callbacks are threadsafe by default
        
        # Checks if the callback is a collision (true) or a separation (false)
        if msg.data:
            self.collisions += 1
            
            if not self.force_mode_enabled:
                print("enabling force mode")
                # Enables force mode
                #self.rtde_c.forceMode(self.task_frame, self.selection_vector, self.wrench, self.force_type, self.limits)
                #self.rtde_c.forceModeSetDamping(self.damping)
                #self.rtde_io.setStandardDigitalOut(0, True)

                self.force_mode_enabled = True
        else:
            self.collisions -= 1

            if self.collisions == 0:
                print("disabling force mode")
                # TODO: Disable force mode
                self.force_mode_enabled = False
                #self.rtde_c.forceModeStop()
                #self.rtde_c.teachMode()

        print(str(self.collisions) + " collisions tracked")

    def main(self):
        rospy.spin()
        #self.rtde_c.forceModeStop()
        self.rtde_c.endTeachMode()

if __name__ == '__main__':
    collision_subscriber = CollisionSubscriber()
    collision_subscriber.main()