#!/usr/bin/env python2.7
import time
import serial
from termcolor import colored
import numpy as np

OUTPUT_BUFFER = '$02,SETM,255,255,255,255,255,255\n'

ser = None
def init(p=None):
  	return openSerial(p)

def openSerial(p=None):
  global ser
# configure the serial connections (the parameters differs on the device you are connecting to)
  if(p == None):
    p = '/dev/ttyUSB0'
  ser = serial.Serial(
    port=p,
    baudrate=19200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
  )

  if (not ser.isOpen()):
    ser.open()
  return ser

def convertToByte(p):
    """
    Converts a float to an integer in the range of 0-512

    :param p: 6DOF pose estimate for LTA3
    :type p: integer
    """
    theOut = 255
    vt = int(np.abs(p)*255)
    if( p > 0.0):
        theOut = int(256 + (vt*.75))
    else:
        theOut = int(255 - (vt*.75))

    #Low power Dead Zone to extend motor life
    if( p < 0.2 and p > -0.2):
        theOut = 255

    #print theOut
    return theOut

def sendOutput(js):
    """
    Generates motor command message to send to MAV1 over xbee

    :param js: array of joint spaces for the MAV1
    :type js: float[]
    """
    #print js.data
    strOut = '$02,SETM'
    for i in range(5):
        p = convertToByte(js.data[i])
        strOut = strOut+',' + str(p)
    strOut = strOut + '\n'
    OUTPUT_BUFFER = strOut

    doMove(OUTPUT_BUFFER)

def doMove(output):
    """
    Sends custom motor command message to MAV1 over xbee

    :param output: Custom motor message
    :type output: string
    """
    global ser
    if(ser == None):
        return 1

    print colored(output,"magenta")

    try:
        return ser.write(output)
    except:
        print colored("serial write error")
