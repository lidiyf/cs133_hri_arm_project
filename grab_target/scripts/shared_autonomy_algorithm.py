#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import Joy
from teleop_lib.controller_interface import ControllerInterface
from std_msgs.msg import Int64
from kortex_driver.msg import *
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

cupsInfo = [{'idx': 0, 'position':[-0.38, 0.33, .1], 'angleToEE': 0, 'probability': 0, 'disToEE': -1, 'actualAngle': 0, 'x_disToEE': -1, 'y_disToEE': -1}, \
			{'idx': 1, 'position':[-0.14, 0.44, 0.1], 'angleToEE': 0, 'probability': 0, 'disToEE': -1, 'actualAngle': 0, 'x_disToEE': -1, 'y_disToEE': -1}, \
			{'idx': 2, 'position':[0.14, 0.44, 0.1], 'angleToEE': 0, 'probability': 0, 'disToEE': -1, 'actualAngle': 0, 'x_disToEE': -1, 'y_disToEE': -1}, \
			{'idx': 3, 'position':[0.38, 0.33, 0.1], 'angleToEE': 0, 'probability': 0, 'disToEE': -1, 'actualAngle': 0, 'x_disToEE': -1, 'y_disToEE': -1}]
userInputAngle = 0
mapInputConstant = 0.5
currGripperPos = [0, 0, 0]
disToEEthreshold = 0.1
finalGoalCupIdx = -1

def map_input_to_displacement(input):
	global mapInputConstant
	return input * mapInputConstant

def update_cups_angle(cupsInfo, userInputAngle, userLinearInput):
	global currGripperPos
	for cup in cupsInfo:
		# x = cup['position'][0] - currGripperPos[0]
		# y = cup['position'][1] - currGripperPos[1]
		# cup['actualAngle'] = math.atan(y / x)
		# cup['angleToEE'] = math.atan(y / x)# - userInputAngle)
		dot = userLinearInput[0]*cup['x_disToEE'] + userLinearInput[1]*cup['y_disToEE']
		det = userLinearInput[0]*cup['y_disToEE'] + userLinearInput[1]*cup['x_disToEE']
		angle = math.atan2(det,dot)
		cup['angleToEE'] = abs(angle)
		# rospy.loginfo(str(cup['idx']) + ": " + cup['angleToEE'])

def update_probability(cupsInfo): # TBA: finish this
	cup_order = [{'cupIdx': 0, 'angleToEE': 0}, \
				{'cupIdx': 1, 'angleToEE': 0}, \
				{'cupIdx': 2, 'angleToEE': 0}, \
				{'cupIdx': 3, 'angleToEE': 0}]
	for cup in cup_order:
		cup['angleToEE'] = cupsInfo[cup['cupIdx']]['angleToEE']
	cup_order = sorted(cup_order, key=get_cup_angle, reverse=False)
	for i in range(4):
		cupsInfo[cup_order[i]['cupIdx']]['probability'] += (2-i) * 0.2
		# print(str(cupsInfo[cup_order[i]['cupIdx']]['idx']) + " "+ str(cupsInfo[cup_order[i]['cupIdx']]['probability']))
		# print(cupsInfo[cup_order[i]['cupIdx']]['probability'])

def update_cup_distance(cupsInfo):
	for cup in cupsInfo:
		xDistance = cup['position'][0] - currGripperPos[0]
		yDistance = cup['position'][1] - currGripperPos[1]
		cup['x_disToEE'] = xDistance
		cup['y_disToEE'] = yDistance
		zDistance = 0 # abs(currGripperPos[2] - cup['position'][2]) # Need this?
		cup['disToEE'] = math.sqrt(xDistance ** 2 + yDistance ** 2 + zDistance ** 2)

def check_threshold(cupsInfo):
	global finalGoalCupIdx
	meetDisThreshold = False
	meetProbThreshold = False
	maxProbCup = sorted(cupsInfo, key=get_cup_prob, reverse=True)[0]
	for cup in cupsInfo:
		print("Probability" + str(cup['idx']) + " " + str(cup['probability']))
	print("Cup with grtest probability: cup" + str(maxProbCup['idx']))
	minDisCup = sorted(cupsInfo, key=get_cup_dis, reverse=False)[0]
	if (maxProbCup == minDisCup):
		finalGoalCupIdx = maxProbCup['idx']
		return maxProbCup['disToEE'] <= disToEEthreshold
	return False



# Helpers
def get_cup_angle(cup):
	return cup['angleToEE']

def get_cup_dis(cup):
	return cup['disToEE']

def get_cup_prob(cup):
	return cup['probability']

def getGripperPosition():
	rospy.loginfo("In getGripperPosition")
	data = rospy.wait_for_message(
			"/my_gen3_lite/base_feedback", BaseCyclic_Feedback)
	# theta_x = np.deg2rad(data.base.tool_pose_theta_x)
	# theta_y = np.deg2rad(data.base.tool_pose_theta_y)
	# theta_z = np.deg2rad(data.base.tool_pose_theta_z) 
	# return [data.base.tool_pose_x, data.base.tool_pose_y, data.base.tool_pose_z]
	return [-data.base.tool_pose_y, data.base.tool_pose_x, data.base.tool_pose_z]
	#, theta_x, theta_y, theta_z]

# Cup Configeration
#	 2	3	 #
# 1 		   4 #
#				#
#	   arm	  #
def reset_probabilities(cupsInfo):
	for cup in cupsInfo:
		cup['probability'] = 0

def main():
	rospy.init_node('algorithm', anonymous=True)
	global cupsInfo
	global currGripperPos
	global finalGoalCupIdx
	global finalGoalPos
	
	print("Hello World")

	controller = ControllerInterface()
	cup_publisher = rospy.Publisher("/cup", Int64, queue_size=10)

	finishedTask = False
	metThreshold = False

	# send the arm home
	# rospy.loginfo("publishing message to move arm home")
	# cup_publisher.publish(0)

	while (not rospy.is_shutdown()) and (not finishedTask):
		# react based on the user's controller input
		buttons = controller.get_user_buttons()
		cmd = controller.get_user_command()
		

		# rospy.loginfo(buttons)
		userLinearInput = [-cmd.linear.y, cmd.linear.x, cmd.linear.z]
		# metThreshold = False
		# finishedTask = False
		noise = 0.01
		# rospy.loginfo("userLinearInput: " + str(userLinearInput))
		# if not finishedTask:
		if not metThreshold: # haven't reach threshold position

			# if the arm goes home, reset the probabilities
			if buttons[0] == 1:
				reset_probabilities(cupsInfo)

			needUpdate = False
			needAngleUpdate = False
			displacement = [0, 0, 0]
			if (abs(userLinearInput[0]) > noise): # map user joystick inpu to a certain value to move the arm
				displacement[0] = map_input_to_displacement(userLinearInput[0])
				needUpdate = True
				needAngleUpdate = True
			if (abs(userLinearInput[1]) > noise):
				displacement[1] = map_input_to_displacement(userLinearInput[1])
				needUpdate = True
				needAngleUpdate = True
			if (abs(userLinearInput[2]) > noise):
				displacement[2] = map_input_to_displacement(userLinearInput[2])
				needUpdate = True
			if (needUpdate): #  if no user input, don't update
				if (needAngleUpdate):
					if userLinearInput[0] == 0:
						userLinearInput[0] += 0.001
					userInputAngle = math.atan(userLinearInput[1] / userLinearInput[0])
					update_cups_angle(cupsInfo, userInputAngle, userLinearInput) # update the angle between each cup and the EE
					update_probability(cupsInfo) # update probability according to angle
					rospy.loginfo("userInputAngle: " + str(userInputAngle * 180/math.pi))
					for cup in cupsInfo:
						rospy.loginfo("cup" + str(cup['idx']) + ": " + str(cup['angleToEE'] * 180/math.pi))

				# TBA:
				# move arm according to displacement
				# update currGripperPos
				currGripperPos = getGripperPosition()
				# rospy.loginfo(currGripperPos)
				update_cup_distance(cupsInfo) # update the distance between each cup and the EE
				metThreshold = check_threshold(cupsInfo) # decide if threshold is meet and should pick up cup
		else:
			# TBA:
			# pick up cup with idx finalGoalCupIdx
			rospy.loginfo("Picking up " + str(finalGoalCupIdx))
			cup_publisher.publish(finalGoalCupIdx+1)
			# return finalGoalCupIdx+1

			finishedTask = True
	rospy.loginfo("Finished Task")
		
if __name__ == '__main__': 
	try: # the try catch block allows this code to easily be interrupted without hanging
		main() # run the main function
	except rospy.ROSInterruptException:
		pass
