# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 15:52:18 2021

@author: Zheyu Zhou
"""


# Set up communication
import sim
import sys
import cv2
import numpy as np
import math
import random
import functools
import transforms3d
import time
#from geometry_msgs.msg import Point

from numpy.linalg import inv
from scipy.linalg import expm, logm
from numpy.linalg import inv, multi_dot, norm
from ece470_lib import *
from usefulFunctions import *
import modern_robotics as mr
print("load success")




######################################################################
######################################################################
######################################################################
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True,
                         5000, 5)  
# Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')
else:
    print('Failed connecting to remote API server')
    sys.exit("Could not Connect")


# Retrieve objects handles
returnCode, UR3_joint1_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_joint1', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for first joint')

returnCode, UR3_joint2_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_joint2', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for second joint')

returnCode, UR3_joint3_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_joint3', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for third joint')

returnCode, UR3_joint4_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_joint4', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')

returnCode, UR3_joint5_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_joint5', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')

returnCode, UR3_joint6_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_joint6', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')

returnCode, UR3_handle = sim.simxGetObjectHandle(
    clientID, 'UR3', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for UR3')

returnCode, UR3_connection_handle = sim.simxGetObjectHandle(
    clientID, 'UR3_connection', sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for UR3_connection_handle')

returnCode, vision_sensor_handle = sim.simxGetObjectHandle(
    clientID, "Vision_sensor", sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for Vision_sensor')

returnCode, resolution, image = sim.simxGetVisionSensorImage(
    clientID, vision_sensor_handle, 0, sim.simx_opmode_blocking)

#sensor_image = np.array(image, dtype = np.uint8)
#sensor_image.resize([resolution[0], resolution[1], 3])


returnCode, floor_handle = sim.simxGetObjectHandle(
    clientID, 'Floor', sim.simx_opmode_blocking)

#############################################################
################# Get Position & Orientation ################
#############################################################

# Get yello ball position
returnCode, yellow_ball_handle = sim.simxGetObjectHandle(
    clientID, 'Yellow', sim.simx_opmode_blocking)
returnCode, yelloposition = sim.simxGetObjectPosition(
    clientID, yellow_ball_handle, -1, sim.simx_opmode_blocking)
YellowPos = np.reshape(yelloposition,(3,1))
# Get brown ball position
returnCode, brown_ball_handle = sim.simxGetObjectHandle(
    clientID, 'Brown', sim.simx_opmode_blocking)
returnCode, brownposition = sim.simxGetObjectPosition(
    clientID, brown_ball_handle, -1, sim.simx_opmode_blocking)
BrownPos = np.reshape(brownposition,(3,1))
# Get green ball position
returnCode, green_ball_handle = sim.simxGetObjectHandle(
    clientID, 'Green', sim.simx_opmode_blocking)
returnCode, greenposition = sim.simxGetObjectPosition(
    clientID, green_ball_handle, -1, sim.simx_opmode_blocking)
GreenPos = np.reshape(greenposition,(3,1))

# Get UR3 Base position
returnCode, baseposition= sim.simxGetObjectPosition(
    clientID, UR3_handle, -1, sim.simx_opmode_blocking)
UR3Pos = np.reshape(baseposition,(3,1))

# Get Yellow Ball Orientation
returnCode, yellow_ball_orientation = sim.simxGetObjectOrientation(clientID, yellow_ball_handle, -1, sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object orientation angles')
R_yellow = transforms3d.euler.euler2mat(yellow_ball_orientation[0], yellow_ball_orientation[1], yellow_ball_orientation[2])

# Get Brown Ball Orientation
returnCode, brown_ball_orientation = sim.simxGetObjectOrientation(clientID, brown_ball_handle, -1, sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object orientation angles')
R_brown = transforms3d.euler.euler2mat(brown_ball_orientation[0], brown_ball_orientation[1], brown_ball_orientation[2])

# Get Green Ball Orientation
returnCode, green_ball_orientation = sim.simxGetObjectOrientation(clientID, green_ball_handle, -1, sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object orientation angles')
R_green = transforms3d.euler.euler2mat(green_ball_orientation[0], green_ball_orientation[1], green_ball_orientation[2])

#############################################################
################# Get Transformation Matrices ###############
#############################################################
Tyellow = np.block([
    [R_yellow[0,:], YellowPos[0,:]],
    [R_yellow[1,:], YellowPos[1,:]],
    [R_yellow[2,:], YellowPos[2,:]],
    [0,0,0,1]
    ])

Tbrown = np.block([
    [R_brown[0,:], BrownPos[0,:]],
    [R_brown[1,:], BrownPos[1,:]],
    [R_brown[2,:], BrownPos[2,:]],
    [0,0,0,1]
    ])

Tgreen = np.block([
    [R_green[0,:], GreenPos[0,:]],
    [R_green[1,:], GreenPos[1,:]],
    [R_green[2,:], GreenPos[2,:]],
    [0,0,0,1]
    ])


################################################################################
#Start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)
################################################################################


#############################################################
######################## theta Computation ##################
#############################################################

print("Start Compute theta")

def computeM():
    # Get the orientation from base to  world frame
    result, orientation = sim.simxGetObjectOrientation(clientID, UR3_connection_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get UR3 end-effector orientation angles for UR3')

    # Get the position from base to world frame
    result, p = sim.simxGetObjectPosition(clientID, UR3_connection_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get UR3 end-effector position for UR3')

    P_initial = np.reshape(p,(3,1))
    R_initial = transforms3d.euler.euler2mat(orientation[0], orientation[1], orientation[2])


    M = np.block([
    [R_initial[0,:], P_initial[0,:]],
    [R_initial[1,:], P_initial[1,:]],
    [R_initial[2,:], P_initial[2,:]],
    [0,0,0,1] ])
    # print ("M", M, "\n")
    return M

def computeS():
    # Set up scew axis with respect to base frame
    result, q1 = sim.simxGetObjectPosition(clientID, UR3_joint1_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object current position for UR3')
    q1 = np.reshape(q1,(3,1))
    a1 = np.array([0, 0, 1])
    #print(-np.cross(a1,q1), "v")
    S1 = toScrew(a1, q1)
    # print ("q1", q1)
    # print ("S1", S1)

    result, q2 = sim.simxGetObjectPosition(clientID, UR3_joint2_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object current position for UR3')
    q2 = np.reshape(q2,(3,1))
    a2 = np.array([[-1],[0],[0]])
    S2 = toScrew(a2, q2)
    # print ("q2", q2)
    # print ("S2", S2)

    result, q3 = sim.simxGetObjectPosition(clientID, UR3_joint3_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object current position for UR3')
    q3 = np.reshape(q3,(3,1))
    a3 = np.array([[-1],[0],[0]])
    S3 = toScrew(a3, q3)
    # print ("q3", q3)
    # print ("S3", S3)

    result, q4 = sim.simxGetObjectPosition(clientID, UR3_joint4_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object current position for UR3')
    q4 = np.reshape(q4,(3,1))
    a4 = np.array([[-1],[0],[0]])
    S4 = toScrew(a4, q4)
    # print ("q4", q4)
    # print ("S4", S4)

    result, q5 = sim.simxGetObjectPosition(clientID, UR3_joint5_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object current position for UR3')
    q5 = np.reshape(q5,(3,1))
    a5 = np.array([[0],[0],[1]])
    S5 = toScrew(a5, q5)
    # print ("q5", q5)
    # print ("S5", S5)

    result, q6 = sim.simxGetObjectPosition(clientID, UR3_joint6_handle, UR3_handle, sim.simx_opmode_blocking)
    if result != sim.simx_return_ok:
        raise Exception('could not get object current position for UR3')
    q6 = np.reshape(q6,(3,1))
    a6 = np.array([[-1],[0],[0]])
    S6 = toScrew(a6, q6)
    # print ("q6", q6)
    # print ("S6", S6)

    S = np.block([[S1, S2, S3, S4, S5, S6]])
    # print ("S", S)
    return S

def computeThetas():
    result1, theta1 = sim.simxGetJointPosition(clientID, UR3_joint1_handle, sim.simx_opmode_blocking)
    if result1 != sim.simx_return_ok:
        raise Exception('could not get first joint variable')
    # print('current value of first joint variable on UR3: theta = {:f}'.format(theta1))

    result2, theta2 = sim.simxGetJointPosition(clientID, UR3_joint2_handle, sim.simx_opmode_blocking)
    if result2 != sim.simx_return_ok:
        raise Exception('could not get second joint variable')
    # print('current value of second joint variable on UR3: theta = {:f}'.format(theta2))

    result3, theta3 = sim.simxGetJointPosition(clientID, UR3_joint3_handle, sim.simx_opmode_blocking)
    if result3 != sim.simx_return_ok:
        raise Exception('could not get third joint variable')
    # print('current value of third joint variable on UR3: theta = {:f}'.format(theta3))

    result4, theta4 = sim.simxGetJointPosition(clientID, UR3_joint4_handle, sim.simx_opmode_blocking)
    if result4 != sim.simx_return_ok:
        raise Exception('could not get fourth joint variable')
    # print('current value of fourth joint variable on UR3: theta = {:f}'.format(theta4))

    result5, theta5 = sim.simxGetJointPosition(clientID, UR3_joint5_handle, sim.simx_opmode_blocking)
    if result5 != sim.simx_return_ok:
        raise Exception('could not get fifth joint variable')
    # print('current value of fifth joint variable on UR3: theta = {:f}'.format(theta5))

    result6, theta6 = sim.simxGetJointPosition(clientID, UR3_joint6_handle, sim.simx_opmode_blocking)
    if result6 != sim.simx_return_ok:
        raise Exception('could not get sixth joint variable')
    # print('current value of sixth joint variable on UR3: theta = {:f}'.format(theta6))

    #Inital thetas from the simulator
    theta = np.array([[theta1], [theta2], [theta3], [theta4], [theta5], [theta6]])
    # theta = np.array([[0.1], [0.1], [0.1], [0.1], [0.1], [0.1]])
    # print (theta)
    return theta


M = computeM()
S = computeS()
theta_initial = computeThetas()
T_2in0 = Tyellow


V = 0.2
while V > 0.115:
    theta_random = np.random.rand(6,1)*2*np.pi-np.pi
    theta_yellow, V = findIK(T_2in0, S, M, theta_random, 100, 0.001, 0.05)
    print(V)
# theta_yellow, V = findIK(T_2in0, S, M, theta_initial, 10000, 0.115, 0.05)
print(theta_initial)
print(theta_random)
print(theta_yellow, "Result")



# THETA_YELLOW, TF = mr.IKinSpace(S, M, Tyellow, np.random.rand(6,1)*2*np.pi-np.pi, 0.01, 0.001)
# print(THETA_YELLOW, "theta")
print("theta computation finished")
#############################################################
################## Move UR3 to Yellow Ball ##################
#############################################################

move1 = sim.simxSetJointPosition(clientID, UR3_joint1_handle, theta_yellow[0], sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint2_handle, theta_yellow[1], sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint3_handle, theta_yellow[2], sim.simx_opmode_streaming)
time.sleep(0.5)

# with theta_initial
# sim.simxSetJointPosition(clientID, UR3_joint4_handle, theta_yellow[3]+ 2*np.pi/3, sim.simx_opmode_streaming)
# time.sleep(0.5)

# with theta_random
sim.simxSetJointPosition(clientID, UR3_joint4_handle, theta_yellow[3], sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint5_handle, theta_yellow[4], sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint6_handle, theta_yellow[5], sim.simx_opmode_streaming)
time.sleep(1)

# manual fix
sim.simxSetJointPosition(clientID, UR3_joint1_handle, theta_yellow[0]+(5)*np.pi/180, sim.simx_opmode_streaming)
time.sleep(0.5)


result, orientation = sim.simxGetObjectOrientation(clientID, UR3_connection_handle, floor_handle, sim.simx_opmode_blocking)
print(orientation, "orientation")

sim.simxSetJointPosition(clientID, UR3_joint4_handle, theta_yellow[3]+(60)*np.pi/180, sim.simx_opmode_streaming)
time.sleep(0.5)

#sim.simxSetJointPosition(clientID, UR3_joint5_handle, (-10)*np.pi/180, sim.simx_opmode_streaming)
time.sleep(0.5)

print("moved to yellow")

'''
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
# =================== Your code starts here ====================#
    l_1 = 0.152
    l_2 = 0.120
    l_3 = 0.244
    l_4 = 0.093
    l_5 = 0.213
    l_6 = 0.083
    l_7 = 0.083
    l_8 = 0.082
    l_9 = 0.0535
    l_10 = 0.059
    x_grip = xWgrip 
    y_grip = yWgrip 
    z_grip = zWgrip

    x_cen = x_grip - l_9*np.cos(np.radians(yaw_WgripDegree))
    y_cen = y_grip - l_9*np.sin(np.radians(yaw_WgripDegree))
    z_cen = z_grip
    dist_xyz_cen = np.sqrt(x_cen**2+y_cen**2)

    theta = np.zeros(6)

    theta[0] = float(np.arctan2(y_cen,x_cen) - np.arcsin((l_6+0.027)/dist_xyz_cen))
    theta[5] = float(np.pi/2 - np.radians(yaw_WgripDegree) + theta[0])



    # dist_xyz_3end = np.cos(np.arcsin((l_6+0.027)/dist_xyz_cen)) * dist_xyz_cen - l_7
    dist_xyz_3end = np.sqrt(dist_xyz_cen**2 - (0.027 + l_6)**2) - l_7
    x_3end = np.cos(theta[0]) * dist_xyz_3end
    y_3end = np.sin(theta[0]) * dist_xyz_3end
    z_3end = z_grip + l_8 + l_10
    

    L = np.sqrt(dist_xyz_3end**2 + (z_3end - l_1)**2)
    alpha = np. arccos((l_3**2 + L**2 - l_5**2)/(2*l_3*L))
    #theta[1] = float(-(np.arctan2(y_3end,x_3end) - np.arccos((l_3**2 - l_5**2 + x_3end**2 + y_3end**2)/(2*l_3*np.sqrt(x_3end**2 + y_3end**2)))))
    theta[1] = float(-alpha - np.arctan2(z_3end - l_1, dist_xyz_3end))
    theta[2] = float(np.pi - np.arccos((l_3**2 + l_5**2 - L**2)/(2*l_3*l_5)))
    theta[3] = float(-theta[2] - theta[1])
    theta[4] = float(-np.pi/2)

    print (float(theta[0]+np.pi),float(theta[1]+85*np.pi/180),float(theta[2]),float(theta[3]-5*np.pi/180), float(theta[4]), float(theta[5]), 'theta')
    #lab_fk(theta[0],theta[1],theta[2],theta[3], theta[4], theta[5])

    return (float(theta[0]+1.5*np.pi),float(theta[1]+85*np.pi/180),float(theta[2]),float(theta[3]-5*np.pi/180), float(theta[4]), float(theta[5]))

'''
    
#############################################################
######################## Close Gripper  #####################
#############################################################
time.sleep(1)
returnCode, gripper_motorhandle = sim.simxGetObjectHandle(
    clientID, "BaxterGripper_closeJoint", sim.simx_opmode_blocking)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for gripper_motorhandle')

print("close gripper")
#sim.simxSetJointTargetVelocity(clientID, gripper_motorhandle, 0.5, sim.simx_opmode_streaming)
sim.simxSetIntegerSignal(clientID, "BaxterGripper_close", 1, sim.simx_opmode_oneshot)
time.sleep(5)
# #############################################################
# ################### Move Ball to Destination  ###############
# #############################################################

time.sleep(2)

move1 = sim.simxSetJointPosition(clientID, UR3_joint1_handle, 0, sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint2_handle, 0, sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint3_handle, 0, sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint4_handle, 0, sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint5_handle, 0, sim.simx_opmode_streaming)
time.sleep(0.5)

sim.simxSetJointPosition(clientID, UR3_joint6_handle, 0, sim.simx_opmode_streaming)

print("Mission Completed")



# returnCode, BarrettHand_attachPoint_handle = sim.simxGetObjectHandle(clientID, 'BarrettHand_attachPoint', sim.simx_opmode_blocking)
# if returnCode != sim.simx_return_ok:
#     raise Exception('could not get object handle for BarrettHand_attachPoint')
    
# returnCode, BarrettHand_attachProxSensor_handle = sim.simxGetObjectHandle(clientID, 'BarrettHand_attachProxSensor', sim.simx_opmode_blocking)
# if returnCode != sim.simx_return_ok:
#     raise Exception('could not get object handle for BarrettHand_attachProxSensor')

# index = 0
# returnCode, shape = sim.simxGetObjects(clientID, sim.sim_object_shape_type,  sim.simx_opmode_blocking)
# print(shape)




# def gripper(clientID, closing, j1, j2):
#     # Get positions of two sides of gripper
#     r, p1 = sim.simxGetJointPosition(clientID, j1, sim.simx_opmode_blocking)
#     r, p2 = sim.simxGetJointPosition(clientID, j2, sim.simx_opmode_blocking)

#     # Close gripper
#     if closing:
#         if p1 < (p2 - 0.008):
#             sim.simxSetJointTargetVelocity(clientID, j1, -0.01, sim.simx_opmode_blocking)
#             sim.simxSetJointTargetVelocity(clientID, j2, -0.04, sim.simx_opmode_blocking)
#         else:
#             sim.simxSetJointTargetVelocity(clientID, j1, -0.04, sim.simx_opmode_blocking)
#             sim.simxSetJointTargetVelocity(clientID, j2, -0.04, sim.simx_opmode_blocking)

#     # Open gripper
#     else:
#         if p1 < p2:
#             sim.simxSetJointTargetVelocity(clientID, j1, 0.04, sim.simx_opmode_blocking)
#             sim.simxSetJointTargetVelocity(clientID, j2, 0.02, sim.simx_opmode_blocking)
#         else:
#             sim.simxSetJointTargetVelocity(clientID, j1, 0.02, sim.simx_opmode_blocking)
#             sim.simxSetJointTargetVelocity(clientID, j2, 0.04, sim.simx_opmode_blocking)