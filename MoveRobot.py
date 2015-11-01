# Constants and functions for moving the robot

import numpy
import gopigo
import time

import SenseLandmarks

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
ENCODER_STEPS_FOR_ABOUT_TURN = 24
ENCODER_TARGET_REACHED = 0

# Default speed
DEFAULT_ROBOT_SPEED = 75
SLEEP_TIME_BETWEEN_STEPS = 0.25

#Movement uncertainty obtained from measurements
MOVEMENT_UNCERTAINTY = numpy.matrix([[0, 0, 0],
                                     [0, 0, 0],
                                     [0, 0, 0]])

# Wait till encoder target is reached
def wait_till_encoder_target_reached():

	time.sleep(SLEEP_TIME_BETWEEN_STEPS)
	while gopigo.read_enc_status() != ENCODER_TARGET_REACHED:
		time.sleep(SLEEP_TIME_BETWEEN_STEPS)

# Go forward by a fixed amount
def go_forward(forward_distance):
	
	gopigo.enable_encoders()
	encoder_steps_required = int(forward_distance / DISTANCE_PER_ENCODER_STEP)
	gopigo.set_speed(DEFAULT_ROBOT_SPEED)
	gopigo.enc_tgt(1, 1, encoder_steps_required)
	gopigo.fwd()
	wait_till_encoder_target_reached()

# Go back by a fixed amount
def go_backwards(backwards_distance):
	
	gopigo.enable_encoders()
	encoder_steps_required = int(backwards_distance / DISTANCE_PER_ENCODER_STEP)
	gopigo.set_speed(DEFAULT_ROBOT_SPEED)
	gopigo.enc_tgt(1, 1, encoder_steps_required)
	gopigo.bwd()
	wait_till_encoder_target_reached()

# Turn in place by the degrees specified. Negative for anti-clockwise.
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

	#Compute the number of encoder steps needed
	encoder_steps_needed = int(ENCODER_STEPS_FOR_ABOUT_TURN * abs(degrees_to_turn) / FULL_REVOLUTION_DEGREES)
	
	#If encoder steps needed are zero, due to truncation, do nothing
	if encoder_steps_needed == 0:
		return
	
	#Turn the number of encoder steps computed
	gopigo.enable_encoders()
	gopigo.enc_tgt(1, 1, abs(encoder_steps_needed))
	if degrees_to_turn > 0:
		gopigo.right_rot()
	else:
		gopigo.left_rot()

	wait_till_encoder_target_reached()

# Go towards the nearest obstacle ahead	
def go_towards_nearest_obstacle():

	# get nearest obstacle
	bearing_to_obstacle, range_to_obstacle = SenseLandmarks.get_nearest_obstacle()

	# Turn towards it
	turn_in_place(bearing_to_obstacle)

	while range_to_obstacle > 2 * ROBOT_LENGTH_CM:

		go_forward(ROBOT_LENGTH_CM)

		# get nearest obstacle
		bearing_to_obstacle, range_to_obstacle = SenseLandmarks.get_nearest_obstacle()

		# Turn towards it
		turn_in_place(bearing_to_obstacle)
