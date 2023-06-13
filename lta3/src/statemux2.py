#!/usr/bin/env python2.7
import rospy
from lta3.msg import state_18DOF
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import copy
#import switch
from termcolor import colored
from statemux import vicon_nofilter, vicon_altitude_ping_nofilter
from helper import switch


class StateMUX(object):

    vicon_subscriber = None
    pozyx_subscriber = None
    oldstates = None

    def __init__(self):
        self.lta3_id = rospy.get_param('~lta3_id', "lta0")
        self.selector_type = rospy.get_param('~selector_type', "vicon_nofilter")
        """Reads in the selector type from the statemux.launch parameter **selector_type** to combine and filter multiple state estimates.

        .. seealso:: Write list of accepted parameters.
        """
        self.vicon_subscriber = rospy.Subscriber("/vicon/" + self.lta3_id + "/" + self.lta3_id, TransformStamped, self.vicon_update)
        """Subscribes to 6 DOF TransformStamped state message from Vicon"""
        self.pozyx_subscriber = rospy.Subscriber("pozyx", TransformStamped, self.pozyx_update)
        """Subscribes to 6 DOF TransformStamped state message from Pozyx tag"""
        self.pozyx_imu_subscriber = rospy.Subscriber("pozyx_imu", Imu, self.pozyx_imu_update)
        """Subscribes to 9 DOF IMU message from pozyx IMU"""
        self.imu_subscriber = rospy.Subscriber("imu", Imu, self.imu_update)
        """Subscribes to 9 DOF IMU message from external IMU"""
        self.altitude_ping_subscriber = rospy.Subscriber("altitude_ping", Float64, self.ping_altitude_update)
        """Subscribes to altitude ping sensor. Returns a float (in meters?)

        .. warning:: We made need to switch this to a FloatStamped custom message type in the future.
        """

        self.state_publisher = rospy.Publisher("state", state_18DOF, queue_size = 10)
        """Publishes 18DOF state message for an individual LTA3"""

        self.vicon = TransformStamped()
        self.old_vicon = TransformStamped()
        self.pozyx = TransformStamped()
        self.pozyx_imu = Imu()
        self.imu = Imu()
        self.ping_altitude = Float64()
        self.old_state = state_18DOF()

    #The following update functions are all the same in format.  They first make a copy of the data, and update the global variables, then they trigger the selector function.

    def vicon_update(self,data):
        """
        Subscribes to new pose estimates from Vicon.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.vicon = copy.deepcopy(data)
        self.selector()

    def pozyx_update(self,data):
        """
        Subscribes to new pose estimates from pozyx tag.

        :param data: 6DOF pose estimate for LTA3
        :type data: TransformStamped.msg
        """
        self.pozyx = copy.deepcopy(data)
        self.selector()

    def pozyx_imu_update(self,data):
        """
        Subscribes to new IMU data from embedded IMU on pozyx tag.

        :param data: 9DOF IMU reading for LTA3
        :type data: Imu.msg
        """
        self.pozyx_imu = copy.deepcopy(data)
        self.selector()

    def imu_update(self,data):
        """
        Subscribes to new IMU data from external IMU.

        :param data: 9DOF IMU reading for LTA3
        :type data: Imu.msg
        """
        self.imu = copy.deepcopy(data)
        self.selector()

    def ping_altitude_update(self,data):
        """
        Subscribes to ping sensor data from onboard altitude ultrasound sensor.

        :param data: Altitude reading
        :type data: Float64.msg
        """
        self.ping_altitude = copy.deepcopy(data)
        self.selector()

    def selector(self):
        """
        Sends all available state data to a selector function to combine and filter.  The selector function is chosen from a the **state_type** filter from a predefined list of functions.
        An 18DOF state estimate for the LTA3 is returned.
        """

        #Consisten Tuple data structure for all state estimates
        states = ((self.pozyx,self.vicon),(self.imu,self.pozyx_imu),(self.ping_altitude))
        state  = state_18DOF()

        switch2 = switch.switch(self.selector_type) #Initialize the switch/case statement for the controller type
        for case in switch2:
            if case('vicon_nofilter'):
                #Estimates Velocity data from Vicon Position Data.  Does not work with Ping yet
                state = vicon_nofilter.MUX(self.old_state,states)
                break
            if case('pozyx_nofilter'):
                print "TODO..."
                break
            if case('vicon_altitude_ping_nofilter'):
                state = vicon_altitude_ping_nofilter.MUX(states)
                break
            if case(): # default, could also just omit condition or 'if True'
                print colored("Selector type not recognized","red")
                # No need to break here, it'll stop anyway

        self.old_vicon = self.vicon
        self.old_state = state

        print colored(("x_vel: " + str(state.velocity.twist.linear.x)),"yellow")
        print colored(("y_vel: " + str(state.velocity.twist.linear.y)),"yellow")
        print colored(("z_vel: " + str(state.velocity.twist.linear.z)),"yellow")

        #print colored(("x:" + str(state.pose.pose.position.x)),"grey")
        #print colored(("y:" + str(state.pose.pose.position.y)),"grey")
        #print colored(("z:" + str(state.pose.pose.position.z)),"grey")
        #print colored(("z:" + str(self.state.pose.pose.position.z)),"cyan")

        self.state_publisher.publish(state)
        print("publishing state estimate...")


if __name__ == '__main__':
    rospy.init_node("STATEMUX")
    statemux = StateMUX()
    rospy.spin()
