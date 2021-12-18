# -*- coding: utf-8 -*-
"""
Created on Mon Nov 29 01:23:37 2021

@author: Zheyu Zhou, Yiyang Xu
"""

import sim
import sys
import cv2
import numpy as np
from scipy.linalg import expm, logm
import time
import matplotlib.pyplot as mlp
from matplotlib.pyplot import imread
PI = np.pi

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    #M = np.eye(4)
	#S = np.zeros((6,6))
    S = np.array([[0,0,1,150,150,0],[0,1,0,-162,0,-150],[0,1,0,-162,0,94],[0,1,0,-162,0,307],[1,0,0,0,162,-260],[0,1,0,-162,0,390]])
    #S = np.array([[0,0,0,0,1,0],[0,1,1,1,0,1],[1,0,0,0,0,0],[150,-162,-162,-162,0,-162],[1,0,0,0,162,-260],[0,1,0,-162,0,390]])
    M = np.array([[0,-1,0,390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
	#S = S.T


	
	# ==============================================================#
    return M, S

def expand_S(S,i):
	S_expanded = np.array([[0,-S[i][2],S[i][1],S[i][3]],[S[i][2],0,-S[i][0],S[i][4]],[-S[i][1],S[i][0],0,S[i][5]],[0,0,0,0]])
	return S_expanded

def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	#theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	#T = np.eye(4)

	M, S = Get_MS()
	inter = np.linalg.multi_dot([expm(expand_S(S,0)*theta1),expm(expand_S(S,1)*theta2),expm(expand_S(S,2)*theta3),expm(expand_S(S,3)*theta4),expm(expand_S(S,4)*theta5),expm(expand_S(S,5)*theta6)])
	T = np.dot(inter,M)




	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
    xgrip = xWgrip 
    ygrip = yWgrip 
    zgrip = zWgrip 
    theta=[0,0,0,0,0,0]
    l01 = 152 
    l02 = 120
    l03 = 244
    l04 = 93
    l05 = 213
    l06 = 83
    l07 = 83
    l08 = 82
    l09 = 0
    l10 = 59 

    xcen = xgrip - l09 * np.cos(np.radians(yaw_WgripDegree))
	
    ycen = ygrip - l09 * np.sin(np.radians(yaw_WgripDegree))
	
    zcen = zgrip 
	
    distcen = np.sqrt(xcen**2 + ycen**2)
	
    theta1 = np.arctan2(ycen,xcen) - np.arcsin((l06+27)/(np.sqrt(xcen**2 + ycen**2)))
	
    theta6 = PI / 2 + theta1 - np.radians(yaw_WgripDegree)
	
    x3end = (np.sqrt(distcen**2 - 110**2)-83)*np.cos(theta1)
	
    y3end = (np.sqrt(distcen**2 - 110**2)-83)*np.sin(theta1)
	
    z3end = zcen + l08 + l10
	
    distend = np.sqrt(x3end**2 + y3end**2)

    theta3 = PI - np.arccos((l03**2 + l05**2 - distend**2 - (z3end-l01)**2)/(2*l03*l05))
	
    theta2 = -np.arcsin(np.sin(PI - theta3)*l05 / np.sqrt(distend**2+(z3end-l01)**2))-np.arctan2((z3end - l01),distend)

    theta4 = - theta2 -theta3 

    theta5 = -PI/2

    #theta=[theta1+ PI,theta2,theta3,theta4- (0.5*PI),theta5,theta6]
    theta=[theta1+PI,theta2+85*PI/180,theta3,-5*PI/180+theta4,theta5,theta6]
    print(theta1/PI*180.0,theta2/PI*180,theta3/PI*180,theta4/PI*180,theta5/PI*180,theta6/PI*180)
    return theta


######################################################################
######################################################################
######################################################################
print ('Program started')
sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print ('Connected to remote API server')
else:
    print('Failed connecting to remote API server')
    sys.exit("Could not Connect")

#############################################################
####################### Get Object Handle  ##################
#############################################################

ur3_handle = np.zeros(6)
for i in range(0,6):
    errorCode, ur3_handle[i] = sim.simxGetObjectHandle(
        clientID,"UR3_joint"+str(i+1),sim.simx_opmode_oneshot_wait)
returnCode=sim.simxSetIntegerSignal(
    clientID,'close_enable',0,sim.simx_opmode_oneshot)
time.sleep(1.5)

returnCode, yellow_ball_handle = sim.simxGetObjectHandle(
    clientID, 'Yellow', sim.simx_opmode_oneshot_wait)

returnCode, green_ball_handle = sim.simxGetObjectHandle(
    clientID, 'Green', sim.simx_opmode_oneshot_wait)

returnCode, brown_ball_handle = sim.simxGetObjectHandle(
    clientID, 'Brown', sim.simx_opmode_oneshot_wait)

returnCode, yellow_des_handle = sim.simxGetObjectHandle(
    clientID, 'YellowD', sim.simx_opmode_oneshot_wait)

returnCode, green_des_handle = sim.simxGetObjectHandle(
    clientID, 'GreenD', sim.simx_opmode_oneshot_wait)

returnCode, brown_des_handle = sim.simxGetObjectHandle(
    clientID, 'BrownD', sim.simx_opmode_oneshot_wait)

print ('Got Object Handle')

#############################################################
####################### Get Ball Position  ##################
#############################################################

# Get yello ball position
returnCode, yellowposition = sim.simxGetObjectPosition(
    clientID, yellow_ball_handle, -1, sim.simx_opmode_oneshot_wait)
yellowposition = np.reshape(yellowposition,(3,1))


returnCode, greenposition = sim.simxGetObjectPosition(
    clientID, green_ball_handle, -1, sim.simx_opmode_oneshot_wait)
greenposition = np.reshape(greenposition,(3,1))

returnCode, brownposition = sim.simxGetObjectPosition(
    clientID, brown_ball_handle, -1, sim.simx_opmode_oneshot_wait)
brownposition = np.reshape(brownposition,(3,1))
print ('Got Ball position')

#############################################################
#################### Get Ball Desitination  #################
#############################################################

# Get yello ball desitination
returnCode, yellowdes = sim.simxGetObjectPosition(
    clientID, yellow_des_handle, -1, sim.simx_opmode_oneshot_wait)
yellowdes = np.reshape(yellowdes,(3,1))
yellowdes = np.array([yellowdes[0],
                     yellowdes[1],
                     [0.033]])

returnCode, greendes = sim.simxGetObjectPosition(
    clientID, green_des_handle, -1, sim.simx_opmode_oneshot_wait)
greendes = np.reshape(greendes,(3,1))
greendes = np.array([greendes[0],
                     greendes[1],
                     [0.033]])


returnCode, browndes = sim.simxGetObjectPosition(
    clientID, brown_des_handle, -1, sim.simx_opmode_oneshot_wait)
browndes = np.reshape(browndes,(3,1))
browndes = np.array([browndes[0],
                     browndes[1],
                     [0.033]])
print ('Got Ball desitination')

# Target = [yellowposition, greenposition, brownposition]
# Destination = [yellowdes, greendes, browndes]

# for m in range(0,3):

#############################################################
####################### Compute theta  ######################
#############################################################

thetaHome = [0,0,0,0,0,0]

thetaYellow = lab_invk(yellowposition[0]*1000, yellowposition[1]*1000, yellowposition[2]*1000, 0)
print(thetaYellow, " Yellow Ball theta")

thetaGreen = lab_invk(greenposition[0]*1000, greenposition[1]*1000, greenposition[2]*1000, 0)
print(thetaGreen, " Green Ball theta")

thetaBrown = lab_invk(brownposition[0]*1000, brownposition[1]*1000, brownposition[2]*1000, 0)
print(thetaBrown, " Brown Ball theta")

thetaYellowDes = lab_invk(yellowdes[0]*1000, yellowdes[1]*1000, yellowdes[2]*1000, 0)
print(thetaYellowDes, " Yellow Ball Destination theta")

thetaGreenDes = lab_invk(greendes[0]*1000, greendes[1]*1000, greendes[2]*1000, 0)
print(thetaGreenDes, " Green Ball Destination theta")

thetaBrownDes = lab_invk(browndes[0]*1000, browndes[1]*1000, browndes[2]*1000, 0)
print(thetaBrownDes, " Brown Ball Destination theta")


#############################################################
###############  Yellow Pick and Place  #####################
#############################################################

    
print("Move to Yellow")
time.sleep(1)

for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaYellow[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    
print("Open Suction Cup")
sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", 1, sim.simx_opmode_oneshot_wait)


print("Approach to Yellow Ball")
time.sleep(1)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[3]),thetaYellow[3]+(8)*np.pi/180,sim.simx_opmode_oneshot_wait)


print("Back to home position")
time.sleep(1)
for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaHome[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)

print("Move to Yellow Destination")
time.sleep(1)

for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaYellowDes[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)

print("Approach to Yellow Ball Destination")
time.sleep(2)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[1]),thetaYellow[1]-(0.45)*np.pi/180,sim.simx_opmode_oneshot_wait)
time.sleep(0.5)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[3]),thetaYellowDes[3]+(12)*np.pi/180,sim.simx_opmode_oneshot_wait)

print("Close Suction Cup")
sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", 0, sim.simx_opmode_oneshot_wait)

print("Back to home position")
time.sleep(1)
for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaHome[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)


#############################################################
###############  Green Pick and Place  #####################
#############################################################

    
print("Move to Green")
time.sleep(1)

for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaGreen[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    
print("Open Suction Cup")
sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", 1, sim.simx_opmode_oneshot_wait)


print("Approach to Green Ball")
time.sleep(2)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[3]),thetaGreen[3]+(8)*np.pi/180,sim.simx_opmode_oneshot_wait)

print("Back to home position")
time.sleep(1)
for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaHome[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)

print("Move to Green Destination")
time.sleep(1)

for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaGreenDes[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    
print("Approach to Green Ball Destination")
time.sleep(2)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[1]),thetaGreenDes[1]-(0.45)*np.pi/180,sim.simx_opmode_oneshot_wait)
time.sleep(0.5)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[3]),thetaGreenDes[3]+(12)*np.pi/180,sim.simx_opmode_oneshot_wait)

print("Close Suction Cup")
sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", 0, sim.simx_opmode_oneshot_wait)

print("Back to home position")
time.sleep(1)
for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaHome[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    

#############################################################
################  Brown Pick and Place  #####################
#############################################################

    
print("Move to Brown")
time.sleep(1)

for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaBrown[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    
print("Open Suction Cup")
sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", 1, sim.simx_opmode_oneshot_wait)


print("Approach to Brown Ball")
time.sleep(2)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[1]),thetaBrown[1]+(1)*np.pi/180,sim.simx_opmode_oneshot_wait)
time.sleep(0.5)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[3]),thetaBrown[3]+(8)*np.pi/180,sim.simx_opmode_oneshot_wait)

print("Back to home position")
time.sleep(1)
for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaHome[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)

print("Move to Brown Destination")
time.sleep(1)

for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaBrownDes[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)
    
print("Approach to Brown Ball Destination")
time.sleep(2)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[0]),thetaBrownDes[0]-(2.75)*np.pi/180,sim.simx_opmode_oneshot_wait)
time.sleep(0.5)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[1]),thetaBrownDes[1]-(4)*np.pi/180,sim.simx_opmode_oneshot_wait)
time.sleep(0.5)
errorCode=sim.simxSetJointTargetPosition(
    clientID,int(ur3_handle[3]),thetaBrownDes[3]+(12)*np.pi/180,sim.simx_opmode_oneshot_wait)

print("Close Suction Cup")
sim.simxSetIntegerSignal(clientID, "BaxterVacuumCup_active", 0, sim.simx_opmode_oneshot_wait)

print("Back to home position")
time.sleep(1)
for k in range(0,6):
    errorCode=sim.simxSetJointTargetPosition(clientID,int(ur3_handle[k]),thetaHome[k],sim.simx_opmode_oneshot_wait)
    time.sleep(0.5)