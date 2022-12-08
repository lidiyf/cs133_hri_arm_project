import math

cupsInfo = [{'idx': 1, 'position':[0, 0, 0], 'angleToEE': 0, 'probability': 0, 'disToEE': -1}, \
			{'idx': 2, 'position':[0, 0, 0], 'angleToEE': 0, 'probability': 0, 'disToEE': -1}, \
			{'idx': 3, 'position':[0, 0, 0], 'angleToEE': 0, 'probability': 0, 'disToEE': -1}, \
			{'idx': 4, 'position':[0, 0, 0], 'angleToEE': 0, 'probability': 0, 'disToEE': -1}]
displacement = [0, 0, 0] # equals to home at the begining
userInputAngle = 0;
mapInputConstant = 0.5
currGripperPos = [0, 0, 0]
disToEEthreshold = 0.1

def map_input_to_displacement(input):
	global mapInputConstant
	return input * mapInputConstant

def update_cups_angle(cupsInfo, userInputAngle):
	global currGripperPos
	for cup in cupsInfo:
		x = cup['position'][0] - currGripperPos[0]
		y = cup['position'][1] - currGripperPos[1]
		cup['angleToEE'] = abs(math.atan(y / x) - userInputAngle)

def update_probability(cupsInfo): # TBA: finish this
	cup_order = [{'cupIdx': 0, 'angleToEE': 0}, \
				{'cupIdx': 1, 'angleToEE': 0}, \
				{'cupIdx': 2, 'angleToEE': 0}, \
				{'cupIdx': 3, 'angleToEE': 0}]
	for cup in cup_order:
		cup['angleToEE'] = cupsInfo[cup['cupIdx']]['angleToEE']
	cup_order.sort(key=get_cup_angle)
	for i in range(4):
		cupsInfo[cup[i][cupIdx]]['probability'] += (i - 1) * 2

def update_cup_distance(cupsInfo):
	for cup in cupsInfo:
		xDistance = abs(currGripperPos[0] - cup['position'][0])
		yDistance = abs(currGripperPos[1] - cup['position'][1])
		zDistance = 0 # abs(currGripperPos[2] - cup['position'][2]) # Need this?
		cup['disToEE'] = math.sqrt(xDistance ** 2 + yDistance ** 2 + zDistance ** 2)

def check_threshold(cupsInfo):
	meetDisThreshold = False
	meetProbThreshold = False
	maxProbCup = cupsInfo.sorted(key=get_cup_prob)
	println("Cup with grtest probability: cup# " + maxProbCup['idx'])
	minDisCup = cupsInfo.sorted(key=get_cup_prob, reverse=True)
	if (maxProbCup == maxProbCup):
		return maxProbCup['disToEE'] <= disToEEthreshold
	return False



# Helpers
def get_cup_angle(cup):
	return cup['angleToEE']

def get_cup_dis(cup):
	return cup['disToEE']

def get_cup_prob(cup):
	return cup['probability']



# Cup Configeration
#     2    3     #
# 1 	       4 #
#                #
#       arm      #

def main():
	global cupsInfo
	global displacement
	global finalGoalPos
	global currGripperPos

	userLinearInput = [0, 0, 0]
	metThreshold = False
	finishedTask = False
	noise = 0.01

	while not finishedTask:
		if not metThreshold: # haven't reach threshold position
			needUpdate = False
			needAngleUpdate = False
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
			if (needUpdate): #  if no user inpu, don't update
				if (needAngleUpdate):
					userInputAngle = math.atan(userLinearInput[1] / userLinearInput[0])
					update_cups_angle(cupsInfo, userInputAngle) # update the angle between each cup and the EE
				update_probability(cupsInfo) # update probability according to angle

				# TBA:
				# move arm according to displacement
				# update currGripperPos

				update_cup_distance(cupsInfo) # update the distance between each cup and the EE
				metThreshold = check_threshold(cupsInfo) # decide if threshold is meet and should pick up cup
		else:
			# TBA:
			# go to position finalGoalPos, and pick up cup

			finishedTask = True
