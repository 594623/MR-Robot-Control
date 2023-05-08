#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from rtde_control import RTDEControlInterface
from rtde_io import RTDEIOInterface

class CollisionSubscriber:
    def __init__(self):
        rospy.init_node("collision_subscriber", anonymous=False)

        self.ip = rospy.get_param("/robot_ip")
        
        self.collisions = 0
        self.force_mode_enabled = False

        self.task_frame = [0, 0, 0, 0, 0, 0]
        self.selection_vector = [1, 1, 1, 0, 0, 0]
        self.wrench = [0, 0, 0, 0, 0, 0]
        self.force_type = 2
        self.limits = [2, 2, 2, 1, 1, 1]
        self.damping = 0.001

        self.force = 10 #N

        # Creates a subscriber for the "/collision" ROS topic
        self.collision_sub = rospy.Subscriber("/collision", Vector3, self.clbk_collision)

        # Connects to the robot
        self.rtde_c = RTDEControlInterface(self.ip)
        #self.rtde_io = RTDEIOInterface(self.ip)

        self.rtde_c.teachMode()
    

    def clbk_collision(self, msg):
        # ROS callbacks are threadsafe by default

        # Checks if the callback is a collision or a separation (if separation, vector is: (0,0,0))
        if msg.x != 0.0 or msg.y != 0.0 or msg.z != 0.0:
            self.collisions += 1
            
            if not self.force_mode_enabled:
                print("enabling force mode\n" + str(msg))
                # Sets wrench values
                self.wrench[0] = self.force * msg.x
                self.wrench[1] = self.force * msg.z
                self.wrench[2] = self.force * msg.y
                # Enables force mode
                self.rtde_c.forceMode(self.task_frame, self.selection_vector, self.wrench, self.force_type, self.limits)
                #self.rtde_c.forceModeSetDamping(self.damping)
                #self.rtde_io.setStandardDigitalOut(0, True)

                self.force_mode_enabled = True
        else:
            self.collisions -= 1

            if self.collisions < 0:
                self.collisions = 0

            if self.collisions == 0:
                print("disabling force mode")
                # TODO: Disable force mode
                self.force_mode_enabled = False
                self.rtde_c.forceModeStop()
                self.rtde_c.teachMode()

        print(str(self.collisions) + " collisions tracked")

    def main(self):
        rospy.spin()
        self.rtde_c.forceModeStop()
        self.rtde_c.endTeachMode()

if __name__ == '__main__':
    collision_subscriber = CollisionSubscriber()
    collision_subscriber.main()