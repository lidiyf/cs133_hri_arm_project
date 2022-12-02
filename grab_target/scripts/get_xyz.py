#!/usr/bin/env python3

# prints the current xyz coordinates of the arm

import kinpy as kp
import math
import rospy
from sensor_msgs.msg import JointState 

# import moveit_msgs.msg


def callback(data):
    chain = kp.build_chain_from_urdf(open("../kinematics_urdf/gen3_lite_arm.urdf").read())

    transform = math.pi / 180.0
    th = {'J0': data.position[0], 'J1': data.position[1],
            'J2': data.position[2], 'J3': data.position[3],
            'J4': data.position[4], 'J5': data.position[5]}

    ret = chain.forward_kinematics(th)["DUMMY"].pos

    rospy.loginfo(ret)
    return ret

def main():
    rospy.init_node('get_xyz', anonymous=True)
    
    # get the coordinates
    currEEPos = rospy.Subscriber("/my_gen3_lite/base_feedback/joint_state", JointState, callback)


    rospy.spin()




# # cup1pos = [0, 0, 0]
# # cup2pos = [0, 0, 0]
# # cup3pos = [0, 0, 0]
# # cup4pos = [0, 0, 0]
# # cup1Prob = 0
# # cup2Prob = 0
# # cup3Prob = 0
# # cup4Prob = 0
# # gripperPos = [0, 0, 0]
# # userInput = [0, 0, 0]
# previousEEPos = [0,0,0]
# endTask = False
# gripperDistanceThreshold = 100
# # gripperDestination = [0, 0, 0]
# meetGripperThreshold = False
# noise = 0.1


# # # cup configeration
# # # 1 2 #
# # # 3 4 #
# # def main():
# 	while not endTask:
# 		if not meetGripperThreshold: # haven't reach threshold position:
# 			# update probability
# 			xInput = currEEPos[0] - previousEEPos[0]
# 			yInput = currEEPos[1] - previousEEPos[1]
# 			zInput = currEEPos[2] - previousEEPos[2]
#             previousEEPos = currEEPos
#             if xInput > noise:
#                 update_probability(cup2Prob, xInput)
#                 update_probability(cup4Prob, xInput)
#             else if xInput < -noise:
#                 update_probability(cup1Prob, xInput)
#                 update_probability(cup3Prob, xInput)
#             if yInput > noise:
#                 update_probability(cup1Prob, yInput)
#                 update_probability(cup2Prob, yInput)
#             else if yInput < -noise:
#                 update_probability(cup3Prob, yInput)
#                 update_probability(cup4Prob, yInput)
# 			if zInput < -noise:
# 				update_probability(max(cup1Prob, cup2Prob, cup3Prob, cup4Prob), zInput)
# 			else if zInput > noise:
# 				update_probability(max(cup1Prob, cup2Prob, cup3Prob, cup4Prob), zInput)

# 			# Move the arm
# 			cupWithMaxProb = max(cup1Prob, cup2Prob, cup3Prob, cup4Prob)
# 			if cupWithMaxProb == cup1Prob:
# 				gripperDestination += (cup1pos - gripperDestination) * 0.1
# 			elif cupWithMaxProb == cup2Prob:
# 				gripperDestination += (cup2pos - gripperDestination) * 0.1
# 			elif cupWithMaxProb == cup3Prob:
# 				gripperDestination += (cup3pos - gripperDestination) * 0.1
# 			else cupWithMaxProb == cup4Prob:
# 				gripperDestination += (cup4pos - gripperDestination) * 0.1

# 			#### Call ros script to run the arm to gripperDestination


# 			# TBA: determine if threshold is meet
# 			# if (...):
# 			# 	meetGripperThreshold = True

# 		else:
# 			# TBA: go and pick up the item

# def update_probability(cubProb, displacement): # update mechanism
# 	cubProb += displacement * 5


if __name__ == '__main__': 
    # example = ExampleMoveItTrajectories()
    try: # the try catch block allows this code to easily be interrupted without hanging
        main() # run the main function
    except rospy.ROSInterruptException:
        pass
