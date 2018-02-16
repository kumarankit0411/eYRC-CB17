############## Task3.2 - Emulation ##############

import vrep
import sys
import time

import math
import numpy as np
import cv2
import cv2.aruco as aruco
import time
from ArUco_library import *
import serial


### configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    bytesize = serial.EIGHTBITS, #number of bits per bytes
    parity = serial.PARITY_NONE, #set parity check: no parity
    stopbits = serial.STOPBITS_ONE #number of stop bits
)
###

################################################### TASK1.2 #######################################################
def choose_goal():
    global goal_pos
    min_distance = 100
    selected_goal = []
    for goal in goal_positions:
        distance = math.sqrt((goal[0]-rob_pos[1][0])**2 + (goal[1]-rob_pos[1][1])**2)
        if distance < min_distance:
            min_distance = distance
            selected_goal = goal
            goal_pos = goal
    chosen_goal = goal_points[goal_positions.index(selected_goal)]
    ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', 1, 'refreshCollection', [chosen_goal, 0], [], [], bytearray(), vrep.simx_opmode_blocking)
    goal_positions.remove(goal_positions[goal_points.index(chosen_goal)])    
    goal_points.remove(chosen_goal)
    return chosen_goal


def set_goal(chosen_goal):
    target_pos = vrep.simxGetObjectPosition(clientID, chosen_goal, -1, vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, goal_dummy_handle, -1, target_pos[1], vrep.simx_opmode_blocking)


def path_from_start_to_goal():
    ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'findPath',  [], [], [], emptyBuff, vrep.simx_opmode_blocking)
    return ret[0]
    

#update search range according to startdummy's position
def update_path_search_range(rob_pos):
    vrep.simxSetObjectPosition(clientID, start_dummy_handle, -1, rob_pos[1], vrep.simx_opmode_oneshot_wait)
    vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'setSearchRange', [], [rob_pos[1][0], rob_pos[1][1]], [], emptyBuff, vrep.simx_opmode_oneshot_wait)


def set_velocity(w_l, w_r):
    #vrep.simxSetJointTargetVelocity(clientID, leftjoint_handle, w_l, vrep.simx_opmode_oneshot)
    #vrep.simxSetJointTargetVelocity(clientID, rightjoint_handle, w_r, vrep.simx_opmode_oneshot)
    ser.write(str([w_l, w_r+15]))

def end_simulation():
    set_velocity(0, 0)
    update_path_search_range([0, start_pos])

def where_is_the_truck():
	#### truck position
	xr = int(Detected_ArUco_markers[9][0][0] + Detected_ArUco_markers[9][2][0])/2   #Actual length in pixel
	yr = int(Detected_ArUco_markers[9][0][1] + Detected_ArUco_markers[9][2][1])/2   #  ''     ''   ''   ''
	pxr = xr*pixel_length   #pixel to meters
	pyr = yr*pixel_width	#  ''  ''   ''   
	npxr = -pxr + 1.5  #fitting to vrep coordinate system
	npyr = pyr - 1.0  #  ''    ''  ''      ''       ''
	
	#### truck orientation
	alpha = 0
	beta = 0
	gamma = angle[9]
	gamma = gamma-180
	gamma = math.radians(gamma)
	
	###resetting dynamics
	ret = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript,'reset',[],[],[],emptyBuff, vrep.simx_opmode_oneshot)

	##changing position and orientation of truck in simulation
	returnCode = vrep.simxSetObjectOrientation(clientID,sparkv_handle,-1,[alpha, beta, gamma],vrep.simx_opmode_oneshot)
	returnCode = vrep.simxSetObjectPosition(clientID,sparkv_handle,-1,[npxr, npyr, 0.0290],vrep.simx_opmode_oneshot)

def crop_frame():
	global top_left_h, bottom_right_h, top_left_w, bottom_right_w, pixel_length, pixel_width	
	while(1):
	    	ret, frame = cap.read()
		if ret==False:
			continue

		frame = frame[top_left_h:bottom_right_h, top_left_w:bottom_right_w]

		Detected_ArUco_markers = detect_ArUco(frame)

		if len(Detected_ArUco_markers) !=3 and len(Detected_ArUco_markers) !=1:
			continue

		top_left_w = int(Detected_ArUco_markers[11][0][0] + Detected_ArUco_markers[11][2][0])/2
		top_left_h = int(Detected_ArUco_markers[11][0][1] + Detected_ArUco_markers[11][2][1])/2
		bottom_right_w = int(Detected_ArUco_markers[0][0][0] + Detected_ArUco_markers[0][2][0])/2
		bottom_right_h = int(Detected_ArUco_markers[0][0][1] + Detected_ArUco_markers[0][2][1])/2
		n = abs(top_left_h - bottom_right_h)
		m = abs(top_left_w - bottom_right_w)
		pixel_length = l/m
		pixel_width = w/n
			
		cv2.imshow('image', frame)
		cv2.waitKey(1)
	
		break

def where_is_the_bot():
	#### Bot position
	xr = int(Detected_ArUco_markers[9][0][0] + Detected_ArUco_markers[9][2][0])/2   #Actual length in pixel
	yr = int(Detected_ArUco_markers[9][0][1] + Detected_ArUco_markers[9][2][1])/2   #  ''     ''   ''   ''
	pxr = xr*pixel_length   #pixel to meters
	pyr = yr*pixel_width	#  ''  ''   ''   
	npxr = -pxr + .45  #fitting to vrep coordinate system
	npyr = pyr - .30  #  ''    ''  ''      ''       ''
	
	#### Bot orientation
	alpha = 0
	beta = 0
	gamma = angle[9]
	gamma = gamma-180
	gamma = math.radians(gamma)

	###resetting dynamics
	ret = vrep.simxCallScriptFunction(clientID,'LuaFunctions',vrep.sim_scripttype_childscript,'reset',[],[],[],emptyBuff, vrep.simx_opmode_oneshot)

	###changing position and orientation of Bot in simulation
	returnCode = vrep.simxSetObjectOrientation(clientID,robot_handle,-1,[alpha, beta, gamma],vrep.simx_opmode_oneshot)
	returnCode = vrep.simxSetObjectPosition(clientID,robot_handle,-1,[npxr, npyr, 0.0500],vrep.simx_opmode_oneshot)

def drive_bot():
    global Detected_ArUco_markers
    global angle, pixel_length, pixel_width
    pos_on_path = 0
    distance = 0
    while(1):
	ret, frame = cap.read()
	frame = frame[top_left_h:bottom_right_h, top_left_w:bottom_right_w]
        Detected_ArUco_markers = detect_ArUco(frame)
	angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
	frame = mark_ArUco(frame,Detected_ArUco_markers,angle)

	if len(Detected_ArUco_markers) !=3 and len(Detected_ArUco_markers) !=1:
		continue
	#where_is_the_truck()
	where_is_the_bot()
	
        rob_pos = vrep.simxGetObjectPosition(clientID, robot_handle, -1, vrep.simx_opmode_oneshot)
        returncode, _, path_pos, _, _ = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', vrep.sim_scripttype_childscript, 'findPathPos',  [], [pos_on_path], [], emptyBuff, vrep.simx_opmode_blocking)
        distance = math.sqrt(path_pos[0]**2 + path_pos[1]**2)
        phi = math.atan2(path_pos[1], path_pos[0])

        
        #this distance allows robot to stop at a safe distance from the goal even if pos_on_path is not 1
        distance_r_g = math.sqrt((rob_pos[1][0]-goal_pos[0])**2+(rob_pos[1][1]-goal_pos[1])**2)
        if distance_r_g < .30:
            v_des = 0
            w_des = 0
            set_velocity(0, 0)
            update_path_search_range(rob_pos)
            break
        
        #partial path condition solved in this if statement
	'''
        if pos_on_path==1 and status == 1:
            status = path_from_start_to_goal()
	    drive_bot()
	    return
	'''

        if(pos_on_path < 1):
            if abs(phi) > .6:
                v_des = 0.01
            else:    
                v_des = 0.08
            w_des = .8 * phi
        else:
            v_des = 0
            w_des = 0
            set_velocity(0, 0)
            update_path_search_range(rob_pos)
            break
        #wheel_seperation = 0.208
	wheel_seperation = 0.18

        v_r = v_des + (wheel_seperation/2) * w_des
        v_l = v_des - (wheel_seperation/2) * w_des

        wheel_diameter = 0.0701
        wheel_radius = wheel_diameter/2

	w_r = int(v_r/wheel_radius*60)
        w_l = int(v_l/wheel_radius*60)

	print w_r, w_l
	if w_r < 80 and w_r > 0:
		w_r = 80
	if w_l < 80 and w_l > 0:
		w_r = 80
	if w_r > -80 and w_r < 0:
		w_r = -80
	if w_l > -80 and w_l < 0:
		w_r = -80
	print w_r, w_l
	
	
        set_velocity(w_l, w_r)
	
	out=''
	while ser.inWaiting() > 0:
            out += ser.read(1)
        if out != '':
            print ">>" + out
        
        if(distance < 0.1):
            pos_on_path = pos_on_path + 0.01

	cv2.imshow('image',frame)
        cv2.waitKey(1)
	

def image_proc():
    while(cap.isOpened()):
        ret, frame = cap.read()
	
	frame = frame[top_left_h:bottom_right_h, top_left_w:bottom_right_w]

        Detected_ArUco_markers = detect_ArUco(frame)

	angle = Calculate_orientation_in_degree(Detected_ArUco_markers)
	
	if len(Detected_ArUco_markers) !=3 and len(Detected_ArUco_markers) !=1:
		continue

	#where_is_the_truck()

	#where_is_the_bot()
	
	drive_bot()
	
        frame = mark_ArUco(frame,Detected_ArUco_markers,angle)
	
        #cv2.namedWindow('image', cv2.WINDOW_NORMAL)
	#cv2.resizeWindow('image', 864, 600)	
        cv2.imshow('image',frame)
        cv2.waitKey(1)

    cap.release()
    cv2.destroyAllWindows()


###################################################################################################################


################ Initialization of handles. Do not change the following section ###################################
if __name__ == '__main__':
	vrep.simxFinish(-1)

	#this line has been changed to allow auto start of simuation through python script
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
	print 'here'
	cylinder_handles=[cylinder_handle1,cylinder_handle2,cylinder_handle3,cylinder_handle4,cylinder_handle5,cylinder_handle6,cylinder_handle7,cylinder_handle8]

	#####################################################################################################################

	print 'here'
	################################################## TASK 1.2 #########################################################
	vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot)
	emptyBuff = bytearray()
	goal_points = []
	goal_positions = []
	start_pos = [0.0, 0.0 ,0.0574]
	set_velocity(0, 0)
	rob_pos = [0, [0.0, 0.0, 0.0574]]
	goal_pos = [0.0, 0.0, 0.0]
	pos_on_path = 0
	distance = 0
	#-----------------
	cap = cv2.VideoCapture(1)
	flag = 0
	l = 1.02 #arena length in meters of the actual arena
	w = 0.94 #arena width in meters of the actual arena
	top_left_w = 0
	top_left_h = 0
	cap.set(3, 1280)
	cap.set(4, 720)
	bottom_right_w = 1280    #actual no of pixels in image in each row
	bottom_right_h = 720     #actual no of pixels in image in each column
	emptyBuff = bytearray()
	Detected_ArUco_markers = 0
	pixel_length = pixel_width = 0
	angle = 0
	#-----------------
	print 'here'
	ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', 1, 'refreshCollection', [cylinder_handle1, 1], [], [], emptyBuff, vrep.simx_opmode_blocking)
	for handle in cylinder_handles:
	    if vrep.simxGetObjectChild(clientID, handle, 0, vrep.simx_opmode_blocking)[1]!=-1:
		goal_points.append(handle)

	del(goal_points[0:2])
	del(goal_points[1:])        
	for goal in goal_points:
	    goal_positions.append(vrep.simxGetObjectPosition(clientID, goal, -1, vrep.simx_opmode_oneshot_wait)[1])

	
	crop_frame()
	while(goal_points!=[]):
		chosen_goal = choose_goal()
		set_goal(chosen_goal)
		status = path_from_start_to_goal()
		drive_bot()
		#move_robot()    #from start to chosen goal
		print 'here'
		#add cylinder to obstacle after robot has achieved that goal
		ret = vrep.simxCallScriptFunction(clientID, 'LuaFunctions', 1, 'refreshCollection', [chosen_goal, 1], [], [], emptyBuff, vrep.simx_opmode_blocking)
	'''	
	except:
	    print("Oops!",sys.exc_info(),"occured.")    
	    end_simulation()     
	'''
	end_simulation()    

	################     Do not change after this #####################

	#end of simulation
	vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

	#####################################################################################################################
