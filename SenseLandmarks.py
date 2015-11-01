# Constants and functions for sensing obstacles

import numpy
import gopigo
import time

#Port that the sbanner is connected to
PORT_A1 = 15

#Scan start, end and step values
SCAN_START = 20
SCAN_END = 160
SCAN_STEP = 10
NUMBER_OF_SCANS = int((SCAN_END - SCAN_START) / SCAN_STEP) + 1
SLEEP_TIME_BETWEEN_STEPS = 0.25
SENSE_LIMIT = 250

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

		print("angle ", angle)
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