#!/usr/bin/env python3

# prints the current xyz coordinates of the arm

import rospy
from sensor_msgs.msg import JointState 


def callback(data):
    rospy.loginfo("Hello World")

def main():
    rospy.init_node('get_xyz', anonymous=True)
    
    # get the coordinates
    sub = rospy.Subscriber("/my_gen3_lite/base_feedback/joint_state", JointState, callback)

    rospy.spin()


if __name__ == '__main__': 
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
