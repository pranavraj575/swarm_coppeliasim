import multiprocessing as mp
import numpy as np

from multiprocessing import shared_memory as sm
from piStreaming._rcv import handle_sensors


class MultiRcv:
    """ Main multiprocessing based object to handle the pi data stream.
        - This class serves as the interface for a user
        - Sensor handling is run on a separate process with shared memory    
    """

    def __init__(self, cfg):
        im_sz = (cfg['im_rows'], cfg['im_cols'], 3)

        # create shared memory for mp
        self.sh_img = sm.SharedMemory(create=True, size=int(np.prod(im_sz)))
        self.sh_bno = sm.SharedMemory(create=True, size=80)
        self.sh_dis = sm.SharedMemory(create=True, size=8)
        self.sh_img_stamp = sm.SharedMemory(create=True, size=8)
        self.sh_bno_stamp = sm.SharedMemory(create=True, size=8)
        self.sh_dis_stamp = sm.SharedMemory(create=True, size=8)

        # interfaces for accessing shared memory
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.bno = np.ndarray(10, dtype=np.double, buffer=self.sh_bno.buf)
        self.dis = np.ndarray(1, dtype=np.double, buffer=self.sh_dis.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)
        self.bno_stamp = np.ndarray(1, np.double, buffer=self.sh_bno_stamp.buf)
        self.dis_stamp = np.ndarray(1, np.double, buffer=self.sh_dis_stamp.buf)

        # create shared memory for shutting sensor process down
        self.sh_flag = sm.SharedMemory(create=True, size=1)
        self.flag = np.ndarray(1, dtype=np.bool_, buffer=self.sh_flag.buf)
        self.flag[0] = True
        
        # initial error to indicate no data
        self.bno[3] = -1.
        self.dis[0] = -1.

        # setup locks for shared memory access
        self.lock_img = mp.Lock()
        self.lock_bno = mp.Lock()
        self.lock_dis = mp.Lock()

        # setup process to handle sensor stream
        names = (self.sh_img.name, self.sh_img_stamp.name,
                 self.sh_bno.name, self.sh_bno_stamp.name,
                 self.sh_dis.name, self.sh_dis_stamp.name,
                 self.sh_flag.name)

        locks = (self.lock_img, self.lock_bno, self.lock_dis)

        args = (cfg, names, locks)
        self.psensor = mp.Process(target=handle_sensors, args=args)

        # start the process
        self.psensor.start()

    def shutdown(self):
        # function to handle memory de-allocation and clean-up
        # - a) call at the end of execution
        # - b) call at SIGINT signal

        # set shutdown flag
        self.flag[0] = False
        self.psensor.join()

        # cleanup shared memory
        self.sh_img.close()
        self.sh_bno.close()
        self.sh_dis.close()
        self.sh_img_stamp.close()
        self.sh_bno_stamp.close()
        self.sh_dis_stamp.close()
        self.sh_flag.close()
        
        self.sh_img.unlink()
        self.sh_bno.unlink()
        self.sh_dis.unlink()
        self.sh_img_stamp.unlink()
        self.sh_bno_stamp.unlink()
        self.sh_dis_stamp.unlink()
        self.sh_flag.unlink()
        
    def get_image(self):
        # grab the image data as np.uint8

        # protected read
        self.lock_img.acquire()
        stamp, img = self.img_stamp, self.img
        self.lock_img.release()
        return stamp, img
        
    def get_bno(self):
        # grab the quaternion, gyro, and linear acceleration

        # protected read
        self.lock_bno.acquire()
        q = self.bno[0:4]
        g = self.bno[4:7]
        a = self.bno[7:]
        stamp = self.bno_stamp
        self.lock_bno.release()
        return stamp, (q, g, a)

    def get_dis(self):
        # grab the distance measurement

        # protected read
        self.lock_dis.acquire()
        stamp, dis = self.dis_stamp, self.dis[0]/100.
        self.lock_dis.release()        
        return stamp, dis

    
