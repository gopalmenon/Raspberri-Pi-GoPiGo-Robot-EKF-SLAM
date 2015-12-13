import MoveRobot
import SenseLandmarks
import gopigo

import numpy

ROBOT_POSE_DIMENSIONS = 3 
MOVEMENT_TYPE_FORWARD = 1 
MOVEMENT_TYPE_ROTATE_IN_PLACE = 2 
INITIAL_ROBOT_POSE = numpy.matrix([0, 0, 0]).T 
INITIAL_ROBOT_LOCATION_UNCERTAINTY = numpy.matrix([[0, 0, 0], [0, 0, 0],[0, 0, 0]])

numberOfLandmarks = 0 
currentRobotPose = INITIAL_ROBOT_POSE 
currentRobotLocationUncertainty = INITIAL_ROBOT_LOCATION_UNCERTAINTY

def getDimensionTranslationMatrix():

	#return numpy.matrix(numpy.pad(numpy.identity(ROBOT_POSE_DIMENSIONS), ((0, 0),(0,3*numberOfLandmarks)), 'constant', constant_values=(0)))
	return numpy.matrix(numpy.identity(ROBOT_POSE_DIMENSIONS))

def getPredictedRobotPose(currentRobotPose, movementType, movementAmount):

	dimensionTranslationMatrix = getDimensionTranslationMatrix()

	newRobotPose = currentRobotPose
	if movementType == MOVEMENT_TYPE_FORWARD:
    		newRobotPose += dimensionTranslationMatrix.T * numpy.matrix([0, movementAmount, 0]).T
	else:
    		newRobotPose += dimensionTranslationMatrix.T * numpy.matrix([0, 0, movementAmount]).T

	return newRobotPose

def getMovementJacobian():

	return numpy.identity(3 * numberOfLandmarks + 3)

def getPredictedRobotUncertainty(currentRobotLocationUncertainty):

	movementJacobian = getMovementJacobian()
	dimensionTranslationMatrix = getDimensionTranslationMatrix()

	return movementJacobian*currentRobotLocationUncertainty*movementJacobian.T + dimensionTranslationMatrix.T * MoveRobot.MOVEMENT_UNCERTAINTY * dimensionTranslationMatrix
