#!/usr/bin/env python3

from teleop_lib.controller_interface import ControllerInterface
from teleop_lib.gen3_movement_utils import Arm
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
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
        rospy.loginfo("going home")
        arm.goto_joint_pose(self.HOME) 


cup_num = -1

def move_to_cup(cup):
    global cup_num
    cup_num = cup.data



def main():
    rospy.init_node('full_autonomy', anonymous=True)
    rospy.loginfo("In main")

    arm = Arm()
    targets = Targets()
    controller = ControllerInterface()

    # move arm to home
    rospy.sleep(0.5)
    arm.goto_joint_pose([370, 345, 75, 90, 80, 275] )
    rospy.sleep(0.5)
    arm.open_gripper()
    rospy.sleep(0.5)

    # move the arm to each joint pose (for testing purposes/ initializing the cup setup)
    # targets.goto_all_targets(arm)
   
    rospy.sleep(3)

    while not rospy.is_shutdown():
        buttons = controller.get_user_buttons() 
        if buttons[0] == 1:
            break

    GOTO_CUP = 'C'

    if GOTO_CUP == 'A':
        arm.goto_joint_pose(targets.A_ABOVE)
        arm.goto_joint_pose(targets.A_END)
        arm.gripper_command(1)
        rospy.sleep(0.5)
        arm.goto_joint_pose(targets.HOME)
    if GOTO_CUP == 'B':
        arm.goto_joint_pose(targets.B_ABOVE)
        arm.goto_joint_pose(targets.B_END)
        arm.gripper_command(1)
        rospy.sleep(0.5)
        arm.goto_joint_pose(targets.HOME)
    if GOTO_CUP == 'C':
        arm.goto_joint_pose(targets.C_ABOVE)
        arm.goto_joint_pose(targets.C_END)
        arm.gripper_command(1)
        rospy.sleep(0.5)
        arm.goto_joint_pose(targets.HOME)
    if GOTO_CUP == 'D':
        arm.goto_joint_pose(targets.D_ABOVE)
        arm.goto_joint_pose(targets.D_END)
        arm.gripper_command(1)
        rospy.sleep(0.5)
        arm.goto_joint_pose(targets.HOME)

            

if __name__ == '__main__': 
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
       
