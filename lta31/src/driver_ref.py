#!/usr/bin/env python2.7
import rospy
from lta3.msg import floats
from helper import switch
from mav1 import MAV1_DRIVER
from termcolor import colored

class Driver_ref:
    """
    Converts joint-space motor commands to data packets that can be read by the onboard LTA3 drivers.
    """

    def __init__(self):
        self.blimp_type = rospy.get_param('~blimp_type', "MAV1")
        """Reads in the blimp type from the IK.launch parameter **blimp_type** """
        self.js_subscriber = rospy.Subscriber("js", floats, self.js_update)
        """Subscribes to 18 DOF des message for an individual LTA3 and activates joint space transcribing loop based of off the blimp_type"""
        self.lta_id = rospy.get_param('~lta3_id', "lta0")
        """Subscribes to 18 DOF des message for an individual LTA3 and activates joint space transcribing loop based of off the blimp_type"""
        self.port = "/dev/xbee" + self.lta_id.replace('lta','') + "B"
        """Assigns xbee port based off of LTA3 id"""

        switch2 = switch.switch(self.blimp_type) #Initialize the switch/case statement for the controller type
        for case in switch2:
            if case('MAV1'):
                MAV1_DRIVER.init(self.port) #change this
                break
            if case('MAV2'):
                print "TODO..."
                break
            if case(): # default, could also just omit condition or 'if True'
                print colored("Selector type not recognized","red")
                # No need to break here, it'll stop anyway


    def js_update(self, data):
        """
        Subscribes to NDOF joint space topics, and converts into lta3 readable data packets. Each lta3 type calls a different
        sendOutput() function.  These are all predefined.

        :param data: NDOF joint space for LTA3
        :type data: Floats.msg
        """

        print data

        switch3 = switch.switch(self.blimp_type) #Initialize the switch/case statement for the controller type
        for case in switch3:
            if case('MAV1'):
                MAV1_DRIVER.sendOutput(data)
                break
            if case('MAV2'):
                print "TODO..."
                break
            if case(): # default, could also just omit condition or 'if True'
                print colored("Selector type not recognized","red")
                # No need to break here, it'll stop anyway

        print "sent to mav1"

if __name__ == '__main__':
    rospy.init_node("DRIVER_REF")
    DRIVER_REF = Driver_ref()
    #blimp.init(xbee_messages.port) #Open Serial Port to send commands to Blimp unit
    rospy.spin()
