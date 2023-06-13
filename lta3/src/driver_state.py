#!/usr/bin/env python2.7
import rospy
from lta3.msg import floats
from termcolor import colored
from helper import switch
import serial
from sensors import pingread


class MAV1_DRIVER_STATE(object):

    raw_state_subscriber = None

    def __init__(self,p=None):
        self.lta3_id = rospy.get_param('~lta3_id', "lta10")
        self.selector_type = rospy.get_param('~selector_type', "ping_altitude")
        """Reads in the selector type from the statemux.launch parameter **selector_type** to combine and filter multiple state estimates.

        Current selector strings that are accepted:

        * **ping-altitude** - Activates a ping node that sends altitude information from a MAV1 or MAV2 to statemux for closed loop control.
        """
        self.raw_state_publisher = rospy.Publisher("raw_state", floats, queue_size = 10)
        """Recieves raw sensor data from an individaul LTA3, and sends an N-DOF raw state message to custom parsing nodes depending on which sensors are active."""

        self.port = "/dev/xbee" + self.lta3_id.replace('lta','') + "B"

        self.ser = self.openSerial(p)
        self.raw_data = [-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0]

    def openSerial(self, p=None):
    # configure the serial connections (the parameters differs on the device you are connecting to)
      if(p == None):
        #p = '/dev/ttyUSB0'
        p = self.port
      self.ser = serial.Serial(
        port=p,
        baudrate=19200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
      )

    def serial_update(self):
        #rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            #voltage = blimp.readSerial() #first read should be voltage
            s = self.ser.readline()

            #print colored(s,"red")
            try:
                s = float(s)
                self.raw_state_publisher.publish([-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,s])
                self.raw_data = [s]
                print self.lta3_id + ":   " + str(s)
            except:
                self.raw_state_publisher.publish([-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0])
                print  self.lta3_id + ":    No Ping Data"
            #right now this only listens for the altitude ping sensor, which is index 10. After a 10DOF IMU

        self.ser.close()


    def selector(self):
        """
        Sends all available state data to a selector function to combine and filter.  The selector function is chosen from a the **state_type** filter from a predefined list of functions.
        An 18DOF state estimate for the LTA3 is returned.
        """

        switch2 = switch.switch(self.selector_type) #Initialize the switch/case statement for the controller type
        for case in switch2:
            if case('ping_altitude'):
                print "collect ping"
                p = pingread.PingRead()
                break
            if case('imu'):
                print "TODO..."
                break
            if case(): # default, could also just omit condition or 'if True'
                print colored("Selector type not recognized","red")
                # No need to break here, it'll stop anyway
    def end():
        if(self.ser == None):
            return 1
        return self.ser.close()

if __name__ == '__main__':
    rospy.init_node("DRIVER_STATE")
    driver_state = MAV1_DRIVER_STATE()
    driver_state.openSerial()
    #Choose which sensors should be listened to
    driver_state.selector()
    try:
        driver_state.serial_update()
    except rospy.ROSInterruptException:
        print "Error."
    rospy.spin()
