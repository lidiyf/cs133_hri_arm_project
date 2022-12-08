#!/usr/bin/env python3

from teleop_lib.controller_interface import ControllerInterface
from teleop_lib.gen3_movement_utils import Arm
from sensor_msgs.msg import Joy
import rospy

class Targets:
    def __init__(self):
        # define all the joint positions for picking up each cup
        self.HOME = [360, 345, 75, 90, 80, 275] 

        self.A_ABOVE = [41, 283, 2, 82, 92, 316]
        self.A_END = [41, 275, 2, 82, 92, 316]

        self.B_ABOVE = [9, 291, 18, 82, 90, 284]
        self.B_END = [9, 283, 18, 82, 90, 284]
        
        self.C_ABOVE = [338, 296, 29, 80, 87, 253]
        self.C_END = [338, 289, 29, 80, 87, 253]

        self.D_ABOVE = [310, 282, 1, 77, 100, 221]
        self.D_END = [310, 275, 1, 77, 100, 221] 

        self.all_targets = self.get_all_targets()

    # move the arm to every joint position (defined in __init__)
    def goto_all_targets(self, arm):
        for position in self.all_targets:
            arm.goto_joint_pose(position)

    # returns an array of all the targets
    def get_all_targets(self):
        return [self.HOME, self.A_ABOVE, self.A_END, self.A_ABOVE, self.B_ABOVE, self.B_END, self.B_ABOVE, self.C_ABOVE, self.C_END, self.C_ABOVE, self.D_ABOVE, self.D_END]

    # returns the xyz coordinates of a given target
    def get_xyz(self, joint_array):
        return joint_array[:3] 

    def goto_home(self, arm):
         arm.goto_joint_pose(self.HOME)



def main():
    rospy.init_node('controller_testing', anonymous=True)

    # create the arm object
    arm = Arm()
    targets = Targets()

    # create the controller
    controller = ControllerInterface()

    # move the arm to each joint pose (for testing purposes/ initializing the cup setup)
    targets.goto_all_targets(arm)
   

    while not rospy.is_shutdown():
        
        # react based on the user's controller input
        buttons = controller.get_user_buttons()
        cmd = controller.get_user_command()

        # rospy.logerr(buttons) 
        # if one of the buttons is pressed
        if buttons[5] == 1:
            arm.gripper_command(0.5)
        elif buttons[4] == 1:
            arm.open_gripper() 
        elif buttons[0] == 1:
            targets.goto_home(arm)

        # record the xyz velocities (for testing purposes)
        rospy.loginfo(str(cmd.linear.x) + ", " + str(cmd.linear.y) + ", " + str(cmd.linear.z))

        # send the user input as an array of velocities
        arm.cartesian_velocity_command([.4*cmd.linear.x, .4*cmd.linear.y, .4*cmd.linear.z, 0,0,0], duration=.1, radians=True)


if __name__ == '__main__': 
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
       
