# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 19:52:02 2017

@author: ankit
"""

## Task 1.2 - Path Planning in V-REP ##
# Import modules

import sys
import vrep



# Write a function here to choose a goal.
def choose_goal():
    pass




# Write a function(s) to set/reset goal and other so that you can iterate the process of path planning
def set_reset_goal():

    pass






# Write a function to create a path from Start to Goal
def path_from_start_to_goal():

    pass






# Write a function to make the robot move in the generated path. 
# Make sure that you give target velocities to the motors here in python script rather than giving in lua.
# Note that the your algorithm should also solve the conditions where partial paths are generated.
def move_robot():
    pos_on_path = 0
    distance = 0
    returnCode = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'findPath',  [], [], [], bytearray(), vrep.simx_opmode_blocking)
    while(1):
        rob_pos = simGetObjectPosition(robot_handle, -1)
        returncode, _, path_pos, _, _ = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'findPathPos',  [], [pos_on_path], [], bytearray(), vrep.simx_opmode_blocking)
        
        '''
        distance = math.sqrt(path_pos[1]^2 + path_pos[2]^2)
        phi = math.atan2(path_pos[2], path_pos[1])

        if(pos_on_path < 1) then
            v_des = 0.1
            w_des = .8 * phi
        else
            v_des = 0
            w_des = 0
            break
        wheel_seperation = 0.208

        v_r = v_des + (wheel_seperation/2) * w_des
        v_l = v_des - (wheel_seperation/2) * w_des

        wheel_diameter = 0.0701
        wheel_radius = wheel_diameter/2

        w_r = v_r/wheel_radius
        w_l = v_l/wheel_radius

        simSetJointTargetVelocity(leftmotor, w_l)
        simSetJointTargetVelocity(rightmotor, w_r)

        if(distance < 0.1) then
            pos_on_path = pos_on_path + 0.01
        '''
            
        









################ Initialization of handles. Do not change the following section ###################################

vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
	print "connected to remote api server"
else:
	print 'connection not successful'
	sys.exit("could not connect")


#####################################################################################################################

# Write your code here
returnCode, a, b, c, d = vrep.simxCallScriptFunction(clientID, 'Script', vrep.sim_scripttype_childscript, 'moveBot',  [], [], [], bytearray(), vrep.simx_opmode_blocking)
print a, b















################     Do not change after this #####################

#end of simulation
#vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
