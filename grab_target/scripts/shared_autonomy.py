#!/usr/bin/env python3

from teleop_lib.controller_interface import ControllerInterface
from teleop_lib.gen3_movement_utils import Arm
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
import moveit_commander

# import numpy as np
import rospy

def main():
    rospy.init_node('controller_testing', anonymous=True)

    arm = Arm()
    arm.home_arm()

    HOME = [360, 345, 75, 90, 80, 275] 

    A_ABOVE = [41, 283, 2, 82, 92, 316]
    A_END = [41, 275, 2, 82, 92, 316]

    B_ABOVE = [9, 291, 18, 82, 90, 284]
    B_END = [9, 283, 18, 82, 90, 284]

    
    C_ABOVE = [338, 296, 29, 80, 87, 253]
    C_END = [338, 289, 29, 80, 87, 253]

    D_ABOVE = [310, 282, 1, 77, 100, 221]
    D_END = [310, 275, 1, 77, 100, 221]



    controller = ControllerInterface()

    rospy.loginfo(arm.get_joints(False))
    arm.goto_joint_pose(HOME) 
    arm.goto_joint_pose(A_ABOVE)
    arm.goto_joint_pose(A_END)

    arm.goto_joint_pose(A_ABOVE)

    arm.goto_joint_pose(B_ABOVE)
    arm.goto_joint_pose(B_END)

    arm.goto_joint_pose(B_ABOVE)

    arm.goto_joint_pose(C_ABOVE)
    arm.goto_joint_pose(C_END)

    arm.goto_joint_pose(C_ABOVE)

    arm.goto_joint_pose(D_ABOVE)
    arm.goto_joint_pose(D_END)
   

    while not rospy.is_shutdown():
        
        buttons = controller.get_user_buttons()
        cmd = controller.get_user_command()

        rospy.logerr(buttons)
        if buttons[5] == 1:
            arm.gripper_command(0.5)
        elif buttons[4] == 1:
            arm.open_gripper() 
        elif buttons[0] == 1:
            arm.home_arm()

        # rospy.loginfo(arm.get_joints(False))
        rospy.loginfo(str(cmd.linear.x) + ", " + str(cmd.linear.y) + ", " + str(cmd.linear.z))

        arm.cartesian_velocity_command([.4*cmd.linear.x, .4*cmd.linear.y, .4*cmd.linear.z, 0,0,0], duration=.1, radians=True)


if __name__ == '__main__': 
    try: # the try catch block allows this code to easily be interrupted without hanging
        print("Hello world")
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
       
