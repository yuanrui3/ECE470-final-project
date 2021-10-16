#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home_2 = np.radians([120, -90, 90, -90, -90, 0])
home = np.radians([167.77,-80.99,103.76,-113.03,-90.17,47.63])

# Hanoi tower location 1
Q11 = [156.43*pi/180.0, -56.13*pi/180.0, 121.6*pi/180.0, -154.67*pi/180.0, -92.65*pi/180.0, 74.14*pi/180.0]
Q12 = [156.45*pi/180.0, -63.95*pi/180.0, 120.9*pi/180.0, -146.6*pi/180.0, -92.73*pi/180.0, 73.57*pi/180.0]
Q13 = [155.58*pi/180.0, -70.78*pi/180.0, 117.92*pi/180.0, -135.72*pi/180.0, -91.23*pi/180.0, 73.37*pi/180.0]

Q21 = [174.22*pi/180.0, -54.83*pi/180.0, 117.7*pi/180.0, -151.37*pi/180.0, -92.31*pi/180.0, 91.95*pi/180.0]
Q22 = [174.26*pi/180.0, -61.86*pi/180.0, 116.65*pi/180.0, -143.32*pi/180.0, -92.32*pi/180.0, 92.0*pi/180.0]
Q23 = [174.25*pi/180.0, -68.29*pi/180.0, 114.41*pi/180.0, -134.65*pi/180.0, -92.32*pi/180.0, 92.0*pi/180.0]

Q31 = [191.24*pi/180.0, -49.57*pi/180.0, 104.85*pi/180.0, -143.23*pi/180.0, -91.90*pi/180.0, 108.33*pi/180.0]
Q32 = [191.22*pi/180.0, -56.12*pi/180.0, 103.75*pi/180.0, -135.57*pi/180.0, -91.90*pi/180.0, 108.37*pi/180.0]
Q33 = [191.07*pi/180.0, -61.32*pi/180.0, 102.10*pi/180.0, -128.73*pi/180.0, -91.90*pi/180.0, 108.27*pi/180.0]
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info, this callback function is called.
"""
def gripper_callback(msg):
	global digital_in_0
	digital_in_0 = msg.DIGIN
	
############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height, vel, acce):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".
    # call move_arm, check gripper feedback
    move_arm(pub_cmd, loop_rate, Q[start_loc-1][2], vel, acce)
    
    
    move_arm(pub_cmd, loop_rate, Q[start_loc-1][start_height-1], vel, acce)
	
    gripper(pub_cmd, loop_rate, suction_on)

    time.sleep(1.0)
	   
    if(digital_in_0==0): 
        gripper(pub_cmd, loop_rate, suction_off)
        move_arm(pub_cmd, loop_rate, home, vel, acce)
        sys.exit()	

    move_arm(pub_cmd, loop_rate, home, vel, acce)
	
    move_arm(pub_cmd, loop_rate, Q[end_loc-1][2], vel, acce)
	
    move_arm(pub_cmd, loop_rate, Q[end_loc-1][end_height-1], vel, acce)
	
    gripper(pub_cmd, loop_rate, suction_off)
	
    move_arm(pub_cmd, loop_rate, home, vel, acce)	

	
    error = 0



    return error


############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	
        ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function
	
    sub_gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, gripper_callback) 

    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 1

    while(not input_done):
        start = raw_input("Enter the starting location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + start + "\n")

        end = raw_input("Enter the ending location <Either 1 2 3 or 0 to quit> ")
        print("You entered " + end + "\n")
        
        start = int (start)
        
        end = int (end)
        
        input_done = 1
        '''
        if(int(input_string) == 1):
            input_done = 1
            loop_count = 1
        elif (int(input_string) == 2):
            input_done = 1
            loop_count = 2
        elif (int(input_string) == 3):
            input_done = 1
            loop_count = 3
        elif (int(input_string) == 0):
            print("Quitting... ")
            sys.exit()
        else:
            print("Please just enter the character 1 2 3 or 0 to quit \n\n")
        '''




    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    third = 0
    for i in range(1,4):
        
        if (i!=start and i!=end):
        
            third = i
            break

    while(loop_count > 0):

        

        #rospy.loginfo("Sending goal 1 ...")
        
		#move_block(pub_command, loop_rate, start_loc, start_height, \
               #end_loc, end_height, 4.0, acce)
        #gripper(pub_command, loop_rate, suction_on)
        move_arm(pub_command, loop_rate, home, 4.0, 4.0)
        move_block(pub_command, loop_rate, start, 1, end, 1, 4.0, 4.0)

        move_block(pub_command, loop_rate, start, 2, third, 1, 4.0, 4.0)

        #move_block(pub_command, loop_rate, end, 1, third, 2, 4.0, 4.0)

        #move_block(pub_command, loop_rate, start, 1, end, 1, 4.0, 4.0)

        #move_block(pub_command, loop_rate, third, 2, start, 1, 4.0, 4.0)

        #move_block(pub_command, loop_rate, third, 1, end, 2, 4.0, 4.0)

        #move_block(pub_command, loop_rate, start, 1, end, 3, 4.0, 4.0)
        # Delay to make sure suction cup has grasped the block
        # time.sleep(1.0)

        #rospy.loginfo("Sending goal 2 ...")

        #rospy.loginfo("Sending goal 3 ...")

        loop_count = loop_count - 1

    gripper(pub_command, loop_rate, suction_off)

	


    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
