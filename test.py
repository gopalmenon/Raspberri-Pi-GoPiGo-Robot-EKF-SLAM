import MoveRobot
import SenseLandmarks
import gopigo
import time

#MoveRobot.turn_in_place(750)
#MoveRobot.go_forward(22.86)
#MoveRobot.go_backwards(100)


print(MoveRobot.go_towards_nearest_obstacle())
print("Voltage ", gopigo.volt())