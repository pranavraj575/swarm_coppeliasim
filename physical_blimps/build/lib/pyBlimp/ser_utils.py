import multiprocessing as mp
import numpy as np
import serial
import signal
import time
from multiprocessing import shared_memory as sm


class serManager:
    """ Class that handles shared multiprocess access to a single serial port
        - used in the multicasting control of the blimp
    """

    def __init__(self):
        self.cmds = []
        self.running = sm.SharedMemory(create=True, size=1)

    def start(self, port, n=1):
        interfaces = []
        for i in range(n):
            self.cmds.append(sm.SharedMemory(create=True, size=10))
            interfaces.append((mp.Lock(), self.cmds[-1].name))

        self.running.buf[0] = True
        args = (port, interfaces, self.running.name)
        self.pManager = mp.Process(target=self.run, args=args)
        self.pManager.start()

        return interfaces
        
    def shutdown(self):
        self.running.buf[0] = False
        self.pManager.join()
        self.running.close()
        self.running.unlink()
        for c in self.cmds:
            c.close()
            c.unlink()

    def run(self, port, interfaces, run_name):
        running = sm.SharedMemory(run_name)

        def handler(signum, frame):
            running.buf[0] = False
            
        # setup control-c handler
        signal.signal(signal.SIGINT, handler)

        # setup serial interface
        ser = create_serial(port)

        # setup sm
        cmds = []
        for iface in interfaces:
            cmds.append((iface[0], sm.SharedMemory(iface[1])))
            
        # spin forever and write cmds
        while running.buf[0]:
            for c in cmds:
                c[0].acquire()
                msg = bytes(c[1].buf[:10])
                b_id = msg[-2]
                if b_id == 255: ser.write(msg[:9])
                else: ser.write(msg)
                c[0].release()                

        # cleanup
        ser.close()
        running.close()
        for c in cmds:
            c[1].close()


def create_serial(port):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 921600
    ser.write_timeout = 10
    ser.open()
    return ser

