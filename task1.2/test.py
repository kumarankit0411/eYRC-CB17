import vrep
import time
import sys
import math

left = [-1, 1, 90]
right = [1, -1, -90]
stop = [0, 0]
forward = [1, 1]


def move(dir):
	if len(dir)==3:
		returncode, orientation = vrep.simxGetObjectOrientation(clientID, robot_handle, -1, vrep.simx_opmode_streaming)
		target_angle = math.degrees(math.atan(orientation[2]))  + dir[2]
		print(target_angle)
		returncode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, dir[0], vrep.simx_opmode_streaming)
		returncode = vrep.simxSetJointTargetVelocity(clientID, rightMotor, dir[1], vrep.simx_opmode_streaming)
		while(math.degrees(math.atan(orientation[2])) > target_angle):
			returncode, orientation = vrep.simxGetObjectOrientation(clientID, robot_handle, -1, vrep.simx_opmode_streaming)
			print math.degrees(math.atan(orientation[2]))
	else:
		returncode = vrep.simxSetJointTargetVelocity(clientID, leftMotor, dir[0], vrep.simx_opmode_streaming)
		returncode = vrep.simxSetJointTargetVelocity(clientID, rightMotor, dir[1], vrep.simx_opmode_streaming)	

	

vrep.simxFinish(-1) #close all open connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # connect to vrep
if clientID!=-1:
	print('Connected to remote API server')
else:
	print('Failed connecting to remote API server')	

returncode = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

returncode, leftMotor = vrep.simxGetObjectHandle(clientID, 'left_joint', vrep.simx_opmode_blocking)
returncode, rightMotor =  vrep.simxGetObjectHandle(clientID, 'right_joint', vrep.simx_opmode_blocking)
returncode, robot_handle = vrep.simxGetObjectHandle(clientID, 'Collector_Bot', vrep.simx_opmode_blocking)


move(forward)

returncode, position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_streaming)

while(position[0]<1):
	returncode, position = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_buffer)
	if returncode==0:
		print position

move(right)		



move(stop)



vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

time.sleep(0.1)		