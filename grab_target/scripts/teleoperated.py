# import numpy as np
import rospy

def main():
    rospy.init_node('controller_teleop', anonymous=True)

    arm = Arm()
    arm.home_arm()


    controller = ControllerInterface()

    while not rospy.is_shutdown():
        # print out the joint locations:
        print(arm.get_joints())
        # goto joint locations:
        # arm.goto_joint_pose()

        buttons = controller.get_user_buttons()
        cmd = controller.get_user_command()

        rospy.logerr(buttons)
        if buttons[5] == 1:
            arm.gripper_command(0.5)
        elif buttons[4] == 1:
            arm.open_gripper() 
        elif buttons[0] == 1:
            arm.home_arm()

        # rospy.loginfo(str(cmd.angular.x) + ", " + str(cmd.angular.y) + ", " + str(cmd.angular.z))

        arm.cartesian_velocity_command([.5*cmd.linear.x, .5*cmd.linear.y, .5*cmd.linear.z, 0,0,0], duration=.1, radians=True)


if __name__ == '__main__': 
    try: # the try catch block allows this code to easily be interrupted without hanging
        print("Hello world")
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
