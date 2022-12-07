#!/usr/bin/env python3

# from math import radians
from teleop_lib.controller_interface import ControllerInterface
from teleop_lib.gen3_movement_utils import Arm
from sensor_msgs.msg import Joy
# import numpy as np
import rospy


def main():
    print("main")
    rospy.init_node('controller_testing', anonymous=True)

    arm = Arm()
    arm.home_arm()
    print("arm created")

    controller = ControllerInterface()

    
    while not rospy.is_shutdown():
        cmd = controller.get_user_command()
        print(cmd)
        arm.cartesian_velocity_command([.5*cmd.linear.x, .5*cmd.linear.y, .5*cmd.linear.z, cmd.angular.z, cmd.angular.x,
                                    cmd.angular.y], duration=.1, radians=True)


if __name__ == '__main__': 
    try: # the try catch block allows this code to easily be interrupted without hanging
        print("Hello world")
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
       
