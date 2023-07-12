import numpy as np
import serial
import struct
import time

from pyBlimp.mixer import *

ser = serial.Serial()
ser.port = "COM9"
ser.baudrate = 921600
ser.write_timeout = 100
ser.open()

u = np.array([0.5, 0.2, -0.3, 0., 0., 0.1])
duty_cycles = mix_inputs(u)
msg = convertCMD(duty_cycles, 1); print(msg)

msgb = b''
for val in msg:
    msgb += struct.pack('!B', int(val))

for t in range(100):
    ser.write(msgb)
    time.sleep(0.001)
    print(str(t)+": wrote", msgb)
