# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 15:52:18 2021

@author: 14677
"""

# Set up communication
import sim
import sys

# import time
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')
else:
    print ('Failed connecting to remote API server')
    sys.exit("Could not Connect")
    
    
# Retrieve objects handles
returnCode, UR3_joint1_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint1', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for first joint')
    
returnCode, UR3_joint2_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint2', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for second joint')
    
returnCode, UR3_joint3_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint3', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for third joint')
    
returnCode, UR3_joint4_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint4', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for fourth joint')
    
returnCode, UR3_joint5_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint5', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for fifth joint')
    
returnCode, UR3_joint6_handle = sim.simxGetObjectHandle(clientID, 'UR3_joint6', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for sixth joint')

returnCode, UR3_base_handle = sim.simxGetObjectHandle(clientID, 'UR3_link1_visible', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for UR3 base')
    
returnCode, UR3_end_handle = sim.simxGetObjectHandle(clientID, 'UR3_link7_visible', sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for UR3_end_handle')
    
returnCode, vision_sensor_handle = sim.simxGetObjectHandle(clientID,"Vision_sensor",sim.simx_opmode_oneshot_wait)
if returnCode != sim.simx_return_ok:
    raise Exception('could not get object handle for Vision_sensor')
    
returnCode,resolution,image=sim.simxGetVisionSensorImage(clientID,vision_sensor_handle,0, sim.simx_opmode_blocking)


returnCode, floor_handle = sim.simxGetObjectHandle(clientID, 'Floor', sim.simx_opmode_oneshot_wait)

# Get yello ball position


returnCode, yellow_ball_handle = sim.simxGetObjectHandle(clientID, 'Yellow', sim.simx_opmode_oneshot_wait)
returnCode, yelloposition = sim.simxGetObjectPosition(clientID, yellow_ball_handle, floor_handle, sim.simx_opmode_streaming)

YelloPos = [-1.075, 0.05, 0.05]
# Get brown ball position
BrownPos = [-1.275, -0.125, 0.05]
# Get green ball position
GreenPos = [-1.225, 0.2, 0.05]

# Move UR3 to ...
# sim.simxSetJointPosition(clientID, UR3_joint1, position, sim.simx_opmode_streaming)