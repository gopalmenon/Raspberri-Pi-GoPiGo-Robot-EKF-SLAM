import MoveRobot
import SenseLandmarks

# Commands that are used to measure robot movement and sensing uncertainty.
# The outputs wil be used to compte the move and sense  uncertainty covariance matrices.

# Constants
NUMBER_OF_ITERATIONS = 15

# Get data for movement forward uncertainty
def calibrate_move():

	count = 0

	while (True):

		count += 1
		if count > NUMBER_OF_ITERATIONS:
			break

		MoveRobot.go_forward(MoveRobot.ROBOT_LENGTH_CM)
		raw_input("Press Enter to continue")

# Get data for rotation uncertainty
def calibrate_turn():

	angle = 5
	count = 0

	while (True):

		count += 1
		if count > NUMBER_OF_ITERATIONS:
			break

		print("Going to turn ", angle, " degrees.")
		MoveRobot.turn_in_place(angle)
		angle += 5
		raw_input("Press Enter to continue")

# Get data for landmark sensing uncertainty
def calibrate_sense():

	
	count = 0

	while (True):

		count += 1
		if count > NUMBER_OF_ITERATIONS:
			break

		obstacle_range = SenseLandmarks.make_sweep()
		print("Obstacle range ", obstacle_range)
		raw_input("Press Enter to continue")
