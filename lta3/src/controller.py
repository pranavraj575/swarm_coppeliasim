#!/usr/bin/env python2.7
import rospy
#catkin_make must be called to build this custom message type
from lta3.msg import state_18DOF
from geometry_msgs.msg import TwistStamped
from termcolor import colored
import copy
from helper import switch
import time

#import sys
#sys.path.append('Controllers')
from mav1 import MAV1_PID
from mav1 import MAV1_VELOCITY

class Controller:
    """
    The Controller node determines which controller to use for an lta3. It subscribes to state and reference setpoints and then publishes an 18DOF desired message.
    """

    state_subscriber = None
    des_publisher = None
    controller = None


    def __init__(self):
        self.controller_type = rospy.get_param('~controller_type', "MAV1_PID")
        """Reads in the controller type from the controller.launch parameter **controller_type** """
        self.update_rate = rospy.get_param('~update_rate', 1)
        print "UPDATE RATE:" + str(self.update_rate)
        """Reads in the update_rate from the controller.launch parameter **update_rate** """
        self.des_publisher = rospy.Publisher("des", TwistStamped, queue_size = 10)
        """Publishes 6DOF desired Force and Torque vectors to the IK node of an individual LTA3"""
        self.state_subscriber = rospy.Subscriber("state", state_18DOF, self.state_update)
        """Subscribes to 18 DOF state message for an individual LTA3 and activates the control loop"""
        self.ref_subscriber = rospy.Subscriber("ref", state_18DOF, self.ref_update)
        """Subscribes to 18 DOF ref message for an individual LTA3.  This is the setpoint used for the feedback loop in the controller send from the AI process."""
        self.state = state_18DOF()
        self.ref = state_18DOF()

        #Temporary default waypoint for debugging
        #self.ref.pose.pose.position.x = 0.0
        #self.ref.pose.pose.position.y = -2.0
        #self.ref.pose.pose.position.z = 1.0

        self.controller = None
        rospy.Rate(10)

    def ref_update(self, data):
        """
        Asyncronously listens for new setpoints from the AI process to use in the control loop.

        .. warning:: Make sure xterm is installed for manual setpoint creation

        :param data: 18DOF ref for LTA3. setpoints from AI proccess
        :type data: lta3.msg
        """
        #use deep copy to update ref message
        self.ref = copy.deepcopy(data)
        print colored(("x:" + str(self.ref.pose.pose.position.x)),"cyan")
        print colored(("y:" + str(self.ref.pose.pose.position.y)),"cyan")
        print colored(("z:" + str(self.ref.pose.pose.position.z)),"cyan")


    def state_update(self, data):
        """
        The main loop that iniates the propper closed-loop controller

        :param data: 18DOF state for LTA3
        :type data: lta3.msg
        """

        self.state = copy.deepcopy(data)


    def des_update(self):
        """
        Publishes desired 18 DOF state messages to the IK node based on state and ref data.  Some vectors can be null because the controller type paramater releates to the blimp type paramater sent to the IK node.

        .. warning:: This could cause issues in the future if the state or ref rate suddenly drops out.  A gated filter (or similar) may be needed.
        """

        #Slect the correct controller from the launch file parameter
        switch2 = switch.switch(self.controller_type) #Initialize the switch/case statement for the controller type
        for case in switch2:
            if case('MAV1_PID'):
                self.controller = MAV1_PID.MAV1_PID()
                print colored('MAV1_PID active',"magenta")
                break
            if case('MAV1_VELOCITY'):
                self.controller = MAV1_VELOCITY.MAV1_VELOCITY()
                break
            if case(): # default, could also just omit condition or 'if True'
                print colored("Controller type not recognized","red")
                # No need to break here, it'll stop anyway

        while not rospy.is_shutdown():
            rate = rospy.Rate(self.update_rate)

            des = self.controller.generate_desired(self.state,self.ref)

            print colored(("x_vel: " + str(self.state.velocity.twist.linear.x)),"yellow")
            print colored(("y_vel: " + str(self.state.velocity.twist.linear.y)),"yellow")
            print colored(("z_vel: " + str(self.state.velocity.twist.linear.z)),"yellow")

            print colored(("x_pos_sp: " + str(self.ref.pose.pose.position.x)),"green")
            print colored(("y_pos_sp: " + str(self.ref.pose.pose.position.y)),"green")
            print colored(("z_pos_sp: " + str(self.ref.pose.pose.position.z)),"green")

            print colored(("x_vel_sp: " + str(self.ref.velocity.twist.linear.x)),"green")
            print colored(("y_vel_sp: " + str(self.ref.velocity.twist.linear.y)),"green")
            print colored(("z_vel_sp: " + str(self.ref.velocity.twist.linear.z)),"green")

            self.des_publisher.publish(des)
            print "publishing...\n"

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("LTA3_Controller")
    LTA3_Controller = Controller()
    #blimp.init(blimp_PID_control.port)
    LTA3_Controller.des_update()
    rospy.spin()
