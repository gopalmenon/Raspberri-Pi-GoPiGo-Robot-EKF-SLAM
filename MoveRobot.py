# Constants and functions for moving the robot

import numpy
import gopigo

# Unit conversion
INCHES_TO_CM = 2.54
FULL_REVOLUTION_DEGREES = 360
HALF_REVOLUTION_DEGREES = FULL_REVOLUTION_DEGREES / 2

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

# Turn in place by the degrees specified. Negative for ancti-clockwise.
def turn_in_place(degrees_to_turn):

	# Turning amount should not be more than 360 degrees
	degrees_to_turn = degrees_to_turn % FULL_REVOLUTION_DEGREES

	if degrees_to_turn == 0:
		return

	# Make turning efficient so that robot does not turn more than half turn
	if abs(degrees_to_turn) > HALF_REVOLUTION_DEGREES:
		if degrees_to_turn > 0:
			degrees_to_turn = -1 * (FULL_REVOLUTION_DEGREES - degrees_to_turn)
		else: 
			degrees_to_turn = FULL_REVOLUTION_DEGREES + degrees_to_turn

	print("Turn degrees", degrees_to_turn)
	