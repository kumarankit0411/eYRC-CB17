## Task 1.2 - Path Planning in V-REP ##
# Import modules

import sys
import vrep
import math



# Write a function here to choose a goal.
def choose_goal():
    min_distance = 100
    selected_goal = []
    for goal in goal_positions:
        distance = math.sqrt((goal[0]-start_pos[0])**2 + (goal[1]-start_pos[1])**2)
        if distance < min_distance:
            min_distance = distance
            selected_goal = goal
    chosen_goal = goal_points[goal_positions.index(selected_goal)]
    ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', 1, 'refreshCollection', [chosen_goal, 0], [], [], emptyBuff, vrep.simx_opmode_blocking)
    goal_points.remove(chosen_goal)
    print goal_points
    return chosen_goal

# Write a function(s) to set/reset goal and other so that you can iterate the process of path planning
def set_goal(chosen_goal):
    target_pos = vrep.simxGetObjectPosition(clientID, chosen_goal, -1, vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, goal_dummy_handle, -1, target_pos[1], vrep.simx_opmode_blocking)






# Write a function to create a path from Start to Goal
def path_from_start_to_goal():
    ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'findPath',  [], [], [], emptyBuff, vrep.simx_opmode_blocking)
    print ret


# Write a function to make the robot move in the generated path. 
# Make sure that you give target velocities to the motors here in python script rather than giving in lua.
# Note that the your algorithm should also solve the conditions where partial paths are generated.
def move_robot():
    pos_on_path = 0
    distance = 0
    path_from_start_to_goal()
    while(1):
        rob_pos = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_oneshot)
        returncode, _, path_pos, _, _ = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'findPathPos',  [], [pos_on_path], [], emptyBuff, vrep.simx_opmode_blocking)
        
        distance = math.sqrt(path_pos[0]**2 + path_pos[1]**2)
        phi = math.atan2(path_pos[1], path_pos[0])

        if(pos_on_path < 1):
            v_des = 0.1
            w_des = .8 * phi
        else:
            v_des = 0
            w_des = 0
            vrep.simxSetJointTargetVelocity(clientID, leftjoint_handle, 0.0, vrep.simx_opmode_oneshot_wait)
            vrep.simxSetJointTargetVelocity(clientID, rightjoint_handle, 0.0, vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, start_dummy_handle, -1, rob_pos[1], vrep.simx_opmode_oneshot_wait)
            break
        wheel_seperation = 0.208

        v_r = v_des + (wheel_seperation/2) * w_des
        v_l = v_des - (wheel_seperation/2) * w_des

        wheel_diameter = 0.0701
        wheel_radius = wheel_diameter/2

        w_r = v_r/wheel_radius
        w_l = v_l/wheel_radius

        vrep.simxSetJointTargetVelocity(clientID, leftjoint_handle, w_l, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightjoint_handle, w_r, vrep.simx_opmode_streaming)

        if(distance < 0.1):
            pos_on_path = pos_on_path + 0.01










################ Initialization of handles. Do not change the following section ###################################

vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID!=-1:
	print "connected to remote api server"
else:
	print 'connection not successful'
	sys.exit("could not connect")

returnCode,robot_handle=vrep.simxGetObjectHandle(clientID,'CollectorBot',vrep.simx_opmode_oneshot_wait)
returnCode,leftjoint_handle=vrep.simxGetObjectHandle(clientID,'left_joint',vrep.simx_opmode_oneshot_wait)
returnCode,rightjoint_handle=vrep.simxGetObjectHandle(clientID,'right_joint',vrep.simx_opmode_oneshot_wait)
returnCode,start_dummy_handle = vrep.simxGetObjectHandle(clientID,'Start',vrep.simx_opmode_oneshot_wait)
returnCode,goal_dummy_handle = vrep.simxGetObjectHandle(clientID,'Goal',vrep.simx_opmode_oneshot_wait)

returnCode,cylinder_handle1=vrep.simxGetObjectHandle(clientID,'Cylinder1',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle2=vrep.simxGetObjectHandle(clientID,'Cylinder2',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle3=vrep.simxGetObjectHandle(clientID,'Cylinder3',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle4=vrep.simxGetObjectHandle(clientID,'Cylinder4',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle5=vrep.simxGetObjectHandle(clientID,'Cylinder5',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle6=vrep.simxGetObjectHandle(clientID,'Cylinder6',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle7=vrep.simxGetObjectHandle(clientID,'Cylinder7',vrep.simx_opmode_oneshot_wait )
returnCode,cylinder_handle8=vrep.simxGetObjectHandle(clientID,'Cylinder8',vrep.simx_opmode_oneshot_wait )

cylinder_handles=[cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5,cylinder_handle6,cylinder_handle7,cylinder_handle8]

#####################################################################################################################

# Write your code here

goal_points = []
goal_positions = []
start_pos = [0,0,0.0574]
emptyBuff = bytearray()

for handle in cylinder_handles:
    if vrep.simxGetObjectChild(clientID, handle, 0, vrep.simx_opmode_blocking)[1]!=-1:
        goal_points.append(handle)
        
for goal in goal_points:
    goal_positions.append(vrep.simxGetObjectPosition(clientID, goal, -1, vrep.simx_opmode_oneshot_wait)[1])

print goal_points
while(goal_points!=[]):
    chosen_goal = choose_goal()
    set_goal(chosen_goal)
    move_robot()
    ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', 1, 'refreshCollection', [chosen_goal, 1], [], [], emptyBuff, vrep.simx_opmode_blocking)
    
#move_robot()














################     Do not change after this #####################

#end of simulation
#vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
