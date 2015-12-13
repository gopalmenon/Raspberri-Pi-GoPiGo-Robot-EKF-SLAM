import MoveRobot
import RobotPosePrediction
import SenseLandmarks
import gopigo
import time
import numpy


#MoveRobot.go_towards_nearest_obstacle()
#MoveRobot.go_forward(22.86)
#MoveRobot.go_backwards(100)


MoveRobot.go_towards_nearest_obstacle()
print("Pose is now ", RobotPosePrediction.currentRobotPose)	
print("Uncertainty is now ", RobotPosePrediction.currentRobotLocationUncertainty)	

print("Voltage ", gopigo.volt())

