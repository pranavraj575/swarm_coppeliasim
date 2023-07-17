#!/usr/bin/env python2.7
from termcolor import colored
import tf
import numpy as np
import math
import rospy

#I tried to import this from the parent class or sibling class but could not figure it out.
from PID import PID

THRUST_FWD_R = 0
THRUST_FWD_L = 1
THRUST_UP_R  = 2
THRUST_UP_L  = 3
THRUST_SIDE  = 4

# Mapping for motors to match right hand rule for MAV1's
#             FWD Left   FWD Right    UP/Down

motor_map                = np.ones(6)
motor_map[THRUST_FWD_R]  = -1.0
motor_map[THRUST_FWD_L]  = -1.0
motor_map[THRUST_UP_R]   = 1.0
motor_map[THRUST_UP_L]   = 1.0
motor_map[THRUST_SIDE]   = 1.0

def setDir(vector,m):
    """
    Adjusts the motor polarity to match the motor mapping on the motor controller. The motor mapping should be the same for each MAV1.

    :param vector: 1-D vector for an individual motor
    :type vector: float
    :param m: Motor mapping calibration
    :type m: float
    :return: Motor power output adjusted for motor controller callibration
    :rtype: float
    """
    return vector*motor_map[m]

def setPower(vector, threshold):
    """
    Sets the power output for an individual motor between -1.0 and 1.0. An additional threshold can be applied to
    reduce current draw and/or provide additional power in a direction. (For ecample, with the mav 1's full power
    in the x direction is double the lateral direction because of motor placement.)

    :param vector: 1-D vector for an individual motor
    :type vector: float
    :param threshold: An additional threshold that can be applied to a motor
    :type threshold: float
    :return: Motor power output between -1.0 and 1.0
    :rtype: float
    """
    if (vector > 1.0):
        vector = 1.0
    if (vector < -1.0):
        vector = -1.0

    vector = vector*threshold
    return vector

def setMove(des):
    """
    The main function that is called by IK.py and converts 18DOF desired vector messages to 5-dimensional joint space.

    :param des: Desired vector for LTA3 to move in
    :type des: lta3.msg
    :return: Array of size 5 for the 5 motors on the mav1
    :rtype: float[]
    """


    x = des.twist.linear.x
    y = des.twist.linear.y
    z = des.twist.linear.z
    theta = des.twist.angular.z

    print "x: " + str(x)
    print "Y: " + str(y)
    print "z: " + str(z)
    print "theta: " + str(theta)

    # x = FWD
    # y = Left
    # z = Up
    k = 1.0
    mot = np.zeros(5)

    mot[THRUST_FWD_R] = x - k*theta
    mot[THRUST_FWD_L] = x + k*theta

    mot[THRUST_UP_R] = z
    mot[THRUST_UP_L] = z

    mot[THRUST_SIDE] = y

    for i in range(5):
      mot[i] = setDir(mot[i],i)

    for i in range(5):
      if (i == 2 or i == 3):
          #higher power for altitude
          mot[i] = setPower(mot[i],0.85)
          print "altitude motor power: " + str(mot[i])
      else:
          mot[i] = setPower(mot[i],.5)

    return mot
