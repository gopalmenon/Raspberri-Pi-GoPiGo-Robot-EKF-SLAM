# Constants and functions for moving the robot

import numpy
import gopigo

# Unit conversion
INCHES_TO_CM = 2.54

# Robot dimensions in inches
ROBOT_WHEEL_DIAMETER_INCHES = 2.5
ROBOT_LENGTH_INCHES = 9
ROBOT_WIDTH_INCHES = 5

# Robot dimensions in cm
ROBOT_WHEEL_DIAMETER_CM = ROBOT_WHEEL_DIAMETER_INCHES * INCHES_TO_CM
ROBOT_LENGTH_CM = ROBOT_LENGTH_INCHES * INCHES_TO_CM
ROBOT_WIDTH_CM = ROBOT_WIDTH_INCHES * INCHES_TO_CM

# Encoder definition
ENCODER_STEPS_PER_REVOLUTION = 18
DISTANCE_PER_ENCODER_STEP = numpy.pi * ROBOT_WHEEL_DIAMETER_CM / ENCODER_STEPS_PER_REVOLUTION

# Default speed
DEFAULT_ROBOT_SPEED = 75

#Movement uncertainty obtained from measurements
MOVEMENT_UNCERTAINTY = numpy.matrix([[0, 0, 0],
                                     [0, 0, 0],
                                     [0, 0, 0]])

# Go forward by a fixed amount
def go_forward(forward_distance):
	encoder_steps_required = int(forward_distance / DISTANCE_PER_ENCODER_STEP)
	gopigo.set_speed(DEFAULT_ROBOT_SPEED)
	gopigo.enc_tgt(1, 1, encoder_steps_required)
	gopigo.fwd()

# Go back by a fixed amount
def go_backwards(backwards_distance):
	encoder_steps_required = int(backwards_distance / DISTANCE_PER_ENCODER_STEP)
	gopigo.set_speed(DEFAULT_ROBOT_SPEED)
	gopigo.enc_tgt(1, 1, encoder_steps_required)
	gopigo.bwd()