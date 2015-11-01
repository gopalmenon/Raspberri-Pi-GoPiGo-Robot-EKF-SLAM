# Constants and functions for sensing obstacles

import numpy
import gopigo
import time

import MoveRobot

#Port that the sbanner is connected to
PORT_A1 = 15

#Scan start, end and step values
SCAN_START = 20
SCAN_END = 160
SCAN_STEP = 10
NUMBER_OF_SCANS = int((SCAN_END - SCAN_START) / SCAN_STEP) + 1
SLEEP_TIME_BETWEEN_STEPS = 0.25
SENSE_LIMIT = 250
SEARCH_LANDMARK_STEP = 45

#Robot heading in terms of servo
ROBOT_HEADING_FOR_SERVO = 90

#Scan uncertainty
SENSE_UNCERTAINTY = numpy.matrix([[0, 0, 0],
                                  [0, 0, 0],
                                  [0, 0, 0]])

# Make a sweep and return distance to obstacles
def make_sweep():
	
	bearing = numpy.zeros(NUMBER_OF_SCANS)
	index = 0
	gopigo.enable_servo()

	#Scan the vicinity in front
	for angle in range(SCAN_START, SCAN_END + SCAN_STEP, SCAN_STEP):

		gopigo.servo(angle)
		time.sleep(SLEEP_TIME_BETWEEN_STEPS)
		distance = gopigo.us_dist(PORT_A1)
		
		#Take into account sense lmits
		if distance < SENSE_LIMIT and distance >= 0:
			bearing[index] = distance
		else:
			bearing[index] = SENSE_LIMIT

		index += 1		

	gopigo.disable_servo()
	return bearing

# Return angle to obstacle given bearing array index
def get_angle_to_obstacle(index):

	servo_angle = SCAN_START + index * SCAN_STEP
	
	# Convert the servo angle to the robot heading angle
	return ROBOT_HEADING_FOR_SERVO - servo_angle

# Return angle and distance to nearest obstacle
def get_nearest_obstacle():

	bearing = make_sweep()
	nearest_obstacle_index = numpy.argmin(bearing)
	if (bearing[nearest_obstacle_index] == SENSE_LIMIT):
		no_obstacles_found = True
	else:
		no_obstacles_found = False

	#Look for an obstacle
	while no_obstacles_found:
		
		# Turn clockwise by 45 degrees to check there
		MoveRobot.turn_in_place(SEARCH_LANDMARK_STEP)

		# Look for obstacles
		bearing = make_sweep()
		nearest_obstacle_index = numpy.argmin(bearing)
		if (bearing[nearest_obstacle_index] == SENSE_LIMIT):
			no_obstacles_found = True
		else:
			no_obstacles_found = False
		
		if no_obstacles_found:
			MoveRobot.go_forward(MoveRobot.ROBOT_LENGTH_CM)

	return get_angle_to_obstacle(nearest_obstacle_index), bearing[nearest_obstacle_index]
	
