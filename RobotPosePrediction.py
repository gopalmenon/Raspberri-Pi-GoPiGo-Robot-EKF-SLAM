import MoveRobot
import SenseLandmarks
import gopigo

import numpy

ROBOT_POSE_DIMENSIONS = 3 
MOVEMENT_TYPE_FORWARD = 1 
MOVEMENT_TYPE_ROTATE_IN_PLACE = 2 
INITIAL_ROBOT_POSE = numpy.matrix([0, 0, numpy.pi / 2]).T 
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
    		newRobotPose += dimensionTranslationMatrix.T * numpy.matrix([movementAmount * numpy.cos(currentRobotPose.item((2, 0))), movementAmount * numpy.sin(currentRobotPose.item((2, 0))), 0]).T
	else:
    		newRobotPose += dimensionTranslationMatrix.T * numpy.matrix([0, 0, currentRobotPose.item((2, 0)) + numpy.pi * movementAmount / MoveRobot.HALF_REVOLUTION_DEGREES]).T

	return newRobotPose

def getMovementJacobian(movementType, movementAmount, currentRobotPose):

	identityMatrix =  numpy.matrix(numpy.identity(3 * numberOfLandmarks + 3))
	if movementType == MOVEMENT_TYPE_FORWARD:
		movementDerivativeWrtPose = numpy.matrix([[0, 0, -1 * movementAmount * numpy.sin(numpy.pi * currentRobotPose.item((2, 0))/MoveRobot.HALF_REVOLUTION_DEGREES)],[0, 0, movementAmount * numpy.cos(numpy.pi * currentRobotPose.item((2, 0))/MoveRobot.HALF_REVOLUTION_DEGREES)],[0, 0, 0]])
		dimensionTranslationMatrix = getDimensionTranslationMatrix()
		return identityMatrix + dimensionTranslationMatrix.T * movementDerivativeWrtPose * dimensionTranslationMatrix
	else:
		return identityMatrix
	

def getPredictedRobotUncertainty(currentRobotLocationUncertainty, movementType, movementAmount, currentRobotPose):

	movementJacobian = getMovementJacobian(movementType, movementAmount, currentRobotPose)
	dimensionTranslationMatrix = getDimensionTranslationMatrix()

	return movementJacobian*currentRobotLocationUncertainty*movementJacobian.T + dimensionTranslationMatrix.T * MoveRobot.MOVEMENT_UNCERTAINTY * dimensionTranslationMatrix
