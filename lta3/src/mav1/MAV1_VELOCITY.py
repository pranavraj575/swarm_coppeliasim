#!/usr/bin/env python2.7
#catkin_make must be called to build this custom message type
from geometry_msgs.msg import TwistStamped
from termcolor import colored
import tf
from PID import PID
import numpy as np
import math
import rospy


class MAV1_VELOCITY(object):

    def __init__(self):
        #altitude, yaw, forward, lateral
        self.yawAngle = 0.0
        self.offset = 0.0  #An offset can be applied to change reference orientation

        #These need to be set
        self.PIDs = [[15, 0, 4], [7, 0,10], [8, 0, 6], [4, 0, 4]]
        self.altitude = PID("Altitude",self.PIDs[0],0.0)
        self.yaw =      PID("Yaw",self.PIDs[1],self.yawAngle)
        self.forward =  PID("Forward",self.PIDs[2],0.0)
        self.lateral =  PID("Lateral",self.PIDs[3],0.0)


    def coordinaterotation(self, vector, yaw):
        """
        rotates a vector by a desired angle

        :param vector: the XY vector for forward and lateral thrust
        :type vector: np.matrix
        :param yaw: the angle needed to rotate from the global frame to the local frame of the mav1
        :type yaw: float
        :return: New rotated forward/lateral vector in local frame
        :rtype: np.matrix
        """
        x = vector[0]
        y = vector[1]
        theta = self.normalizeAngle(yaw)

        rot = np.matrix( ((math.cos(theta),1*math.sin(theta)), (-1*math.sin(theta), math.cos(theta)) )) # Clockwise rotation
        coord = np.matrix([[x],[y]]) #current X,Y position
        coordprime = rot*coord #Rotated vector
        return coordprime

    # Adjust angles to stay within pi and -pi
    def normalizeAngle(self, angle):
        """
        Normalizes angle to be within the range (-3.14,3.14)

        :param angle: angle, in radians
        :type angle: float
        :return: Normalized angle, in radians
        :rtype: float
        """
        newAngle = angle
        while (newAngle <= -3.14):
             newAngle = newAngle + 3.14
        while (newAngle > 3.14):
             newAngle = newAngle - 3.14
        return newAngle

    def generate_desired(self,state,ref):

        self.forward.setSetPoint(ref.velocity.twist.linear.x)
        self.lateral.setSetPoint(ref.velocity.twist.linear.y)
        self.altitude.setSetPoint(ref.velocity.twist.linear.z)

        """
        Starts the control loop for MAV1's. Four individual PIDs are used for forward, lateral, altitude, and yaw movement.

        ``**Note:** The overall gain affects the "power" of each DOF.  For MAV1's the lateral prop generates significantly less thrust than the 2 forward facing thrusters.``

        :param state: Current state of LTA3
        :type state: lta3.msg
        :param ref: Most recent setpoint for LTA3
        :type ref: lta3.msg
        :return: 18 DOF desired reference vectors for controller.py to send to the IK node
        :rtype: lta3.msg
        """

        pitch, roll, yaw = tf.transformations.euler_from_quaternion((state.pose.pose.orientation.x,
                                                                        state.pose.pose.orientation.y,
                                                                        state.pose.pose.orientation.z,
                                                                        state.pose.pose.orientation.w))

        print "Yaw: " + str(yaw)
        yawcorrection = -1*self.yaw.update(yaw)
        forwardcorrection = self.forward.update(state.velocity.twist.linear.x)
        lateralcorrection = self.lateral.update(state.velocity.twist.linear.y)
        altitudecorrection = self.altitude.update(state.velocity.twist.linear.z)

        # Rotate desired vector from global frame to local frame
        v = [forwardcorrection, lateralcorrection]
        vrot = self.coordinaterotation(v,(self.yawAngle-yaw))
        forwardcorrection = vrot.item(0,0)
        lateralcorrection = vrot.item(1,0)


        des = TwistStamped()

        des.twist.linear.x = .5 * forwardcorrection
        des.twist.linear.y = lateralcorrection
        des.twist.linear.z = altitudecorrection
        des.twist.angular.z = yawcorrection

        #des.child_frame_id = "des"
        des.header.stamp = rospy.Time.now()

        print colored("Altitude Correction: " + str(altitudecorrection),"cyan")
        print colored("Forward Correction: " + str(forwardcorrection),"cyan")
        print colored("Lateral Correction: " + str(lateralcorrection),"cyan")
        print colored("Yaw Correction: " +     str(yawcorrection),"cyan")

        return des
