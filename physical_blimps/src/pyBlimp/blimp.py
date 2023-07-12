import multiprocessing as mp
import numpy as np
import signal
import time

from multiprocessing import shared_memory as sm
from pyBlimp._controller import handle_controller
from pyBlimp.utils import *
from pyBlimp.ser_utils import serManager

class BlimpManager:
    def __init__(self, cfg, port, logger=False):
        N = len(cfg)

        # generate a single serial manager
        self._S = serManager()
        interfaces = self._S.start(port, N)

        # generate all blimps
        self._blimps = []
        for n in range(N):
            print("Creating blimp", n)
            self._blimps.append(Blimp(interfaces[n], cfg[n], logger=logger))

        # calibrate all blimps
        for n in range(N):
            print("Calibrating", n)
            self.zero_xy_rot(n)
            self.zero_z_rot(n)

    def shutdown(self):
        # shutdown blimps
        for b in self._blimps:
            b.shutdown()

        # shutdown serial
        self._S.shutdown()

    def get_running(self, idx=0):
        return self._blimps[idx].get_running()

    def get_image(self, idx=0):
        return self._blimps[idx].get_image()

    def get_state(self, idx=0):
        return self._blimps[idx].get_state()

    def get_euler(self, idx=0):
        return self._blimps[idx].get_euler()

    def get_veuler(self, idx=0):
        return self._blimps[idx].get_veuler()

    def get_accel(self, idx=0):
        return self._blimps[idx].get_accel()

    def get_alt(self, idx=0):
        return self._blimps[idx].get_alt()

    def set_extra(self, name, extra, idx=0):
        return self._blimps[idx].set_extra(name, extra)

    def set_des(self, des, idx=0):
        return self._blimps[idx].set_des(des)

    def set_cmd(self, cmd, idx=0):
        return self._blimps[idx].set_cmd(cmd)

    def zero_xy_rot(self, idx=0):
        return self._blimps[idx].zero_xy_rot()
        
    def zero_z_rot(self, idx=0):
        return self._blimps[idx].zero_z_rot()


class Blimp:
    def __init__(self, ser, cfg, motors_only=False, logger=False):
        """ Main interfacing class for a user to control a single blimp
            - ser is a serManager interface (lock and sm to write cmd to)
            - cfg is a dict containing parameters for starting the blimp
        """
        
        # define incoming image size
        im_sz = (cfg['im_rows'], cfg['im_cols'], 3)

        # setup shared memory state (1-z, 4-quat, 3-euler, 3-linear accel)
        self.lock_x = mp.Lock()
        self.sh_x = sm.SharedMemory(create=True, size=80)
        self.sh_x_stamp = sm.SharedMemory(create=True, size=8)
        self.x = np.ndarray(10, dtype=np.double, buffer=self.sh_x.buf)
        self.x_stamp = np.ndarray(1, np.double, buffer=self.sh_x_stamp.buf)
        self.x[:] = 0.
        self.x_stamp[0] = 0.        

        # setup shared image access
        self.lock_I = mp.Lock()
        self.sh_img = sm.SharedMemory(create=True, size=int(np.prod(im_sz)))        
        self.sh_img_stamp = sm.SharedMemory(create=True, size=8)
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)
        self.img[:,:,:] = 0
        self.img_stamp[0] = 0.

        # setup shared desired states (z, roll, pitch, yaw)
        self.lock_des = mp.Lock()
        self.sh_des = sm.SharedMemory(create=True, size=32)
        self.des = np.ndarray(4, dtype=np.double, buffer=self.sh_des.buf)
        self.des[:] = [0., 0., 0., 1.]

        # setup shared rotation zeros
        self.lock_rot0 = mp.Lock()
        self.sh_rot0 = sm.SharedMemory(create=True, size=24)
        self.rot0 = np.ndarray(3, dtype=np.double, buffer=self.sh_rot0.buf)

        # setup shared manual mode and user interface
        self.lock_cmd = mp.Lock()
        self.sh_man = sm.SharedMemory(create=True, size=1)
        self.man = np.ndarray(1, dtype=np.bool_, buffer=self.sh_man.buf)
        self.sh_cmd = sm.SharedMemory(create=True, size=32)
        self.cmd = np.ndarray(4, dtype=np.double, buffer=self.sh_cmd.buf)        
        self.man[0] = True
        self.cmd[:] = np.zeros(4)

        # setup shared process flag
        self.sh_run = sm.SharedMemory(create=True, size=1)
        self.run = np.ndarray(1, dtype=np.bool_, buffer=self.sh_run.buf)
        self.run[0] = True
        self.sh_reported = sm.SharedMemory(create=True, size=1)

        # start the controller process
        locks = (self.lock_x, self.lock_I, self.lock_des, self.lock_rot0, self.lock_cmd)
             
        names = (self.sh_x.name, self.sh_x_stamp.name, self.sh_img.name, 
                 self.sh_img_stamp.name, self.sh_des.name, self.sh_rot0.name,
                 self.sh_man.name, self.sh_cmd.name, self.sh_run.name,
                 self.sh_reported.name)

        extras = ()
        self.lock_extra = dict()
        self.sh_extra = dict()
        self.extra = dict()

        if "extra_names" in cfg:
            for idx in range(len(cfg["extra_names"])):
                lock  = mp.Lock()
                name  = cfg["extra_names"][idx]
                size  = cfg["extra_sizes"][idx]
                dtype = cfg["extra_types"][idx]
                sm_size = size
                if dtype == "float32":
                    dtype = np.float32
                    sm_size *= 4

                if dtype == "bool":
                    dtype = np.bool_

                self.lock_extra[name] = lock
                self.sh_extra[name] = sm.SharedMemory(create=True, size=sm_size)
                self.extra[name] = np.ndarray(size, dtype=dtype, buffer=self.sh_extra[name].buf)
                
                sm_name = self.sh_extra[name].name
                extras += ((name, sm_name, size, dtype, lock),)

        args = (ser, cfg, locks, names, extras, logger)
        self.pcontroller = mp.Process(target=handle_controller, args=args)
        self.pcontroller.start()

    # shutdown operation
    def shutdown(self):
        # set to zeros to keep from flying away
        self.set_cmd([0.,0.,0.,0.])

        # set shutdown flag
        self.run[0] = False
        print("Shutting down blimp")
        self.pcontroller.join()
        print("joined!")
        # cleanup shared memory
        self.sh_x.close()
        self.sh_x_stamp.close()
        self.sh_img.close()
        self.sh_img_stamp.close()
        self.sh_des.close()
        self.sh_rot0.close()
        self.sh_man.close()
        self.sh_cmd.close()
        self.sh_run.close()
        self.sh_reported.close()

        self.sh_x.unlink()
        self.sh_x_stamp.unlink()
        self.sh_img.unlink()
        self.sh_img_stamp.unlink()
        self.sh_des.unlink()
        self.sh_rot0.unlink()
        self.sh_man.unlink()
        self.sh_cmd.unlink()
        self.sh_run.unlink()
        self.sh_reported.unlink()

        for key in self.sh_extra:
            self.sh_extra[key].close()
            self.sh_extra[key].unlink()

    # user-interface
    def get_running(self):
        return self.run[0]

    def get_image(self):
        # protected read
        self.lock_I.acquire()
        I_ = self.img.copy()
        self.lock_I.release()
        return I_/255.

    def get_state(self):
        """ get the full corrected state
            - returns in local blimp coordinate frame
            - returns (pz, r, p, yw, vr, vp, vyw, ax, ay, az)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()    
        return x_

    def get_euler(self):
        """ get the linear acceleration (without gravity)
            - returns in local blimp coordinate frame
            - returns (r, p, yw)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()
        return x_[1:4]

    def get_veuler(self):
        """ get the angular velocity
            - returns in local blimp coordinate frame
            - returns (vr, vp, vyw)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()
        return x_[5:8]    

    def get_accel(self):
        """ get the linear acceleration (without gravity)
            - returns in local blimp coordinate frame
            - returns (ax, ay, az)
        """
        # protected read
        self.lock_x.acquire()
        x_ = self.x.copy()
        self.lock_x.release()
        return x_[8:]

    def get_alt(self):
        """ get the roll/pitch corrected altitude
            - returns in local blimp coordinate frame
            - uses a write-safe lock to get the altitude
        """
        # protected read
        eul = self.get_euler()

        # correct altitude and return
        return x_[0]*np.cos(eul[0])*np.cos(eul[1])

    def set_extra(self, name, extra):
        """ sets additional data for the blimp to time-sync for post-processing
            - name is a string-key matching the config file
            - extra is a np array of correct size and type            
        """
        # protected write                        
        self.lock_extra[name].acquire()
        self.extra[name][:] = extra
        self.lock_extra[name].release()

    def set_des(self, des):
        """ set the desired local frame states for the blimp to track
            - des is a numpy array of dim 4x1 with values as
            - des[0] : desired roll
            - des[1] : desired pitch
            - des[2] : desired yaw
            - des[3] : desired altitude
            
        """
    
        # limit the desired states to appropriate space    
        des[3] = np.clip(des[3], 0.0, 2.5)
        des[0:3] = wrap(des[0:3])
        
        # engage auto mode (no lock needed)
        self.man[0] = False
        
        # protected write to desired state
        self.lock_des.acquire()
        self.des[:] = des
        self.lock_des.release()

    def set_cmd(self, cmd):
        """ set the manual inputs to apply
            - cmd is a numpy array of dim 4x1 with values as
            - cmd[0] : force along x
            - cmd[1] : force along y
            - cmd[2] : force along z
            - cmd[3] : torque along z
        """

        # engage manual mode (no lock needed)
        self.man[0] = True

        # protected write to manual input
        self.lock_cmd.acquire()
        self.cmd[:] = cmd
        self.lock_cmd.release()        

    # setup functions
    def zero_xy_rot(self):
        """ grabs the current roll and pitch angle and stores it as
            - the zero angle which is used to correct all measurements
            - for user interfacing and for blimp stabilization
        """

        # wait until data has been reported
        while not self.sh_reported.buf[0] and self.run[0]: pass

        # protected read
        [r, p, _] = self.get_euler()

        # protected write
        self.lock_rot0.acquire()
        self.rot0[:2] = [r, p]
        self.lock_rot0.release()
        
    def zero_z_rot(self):
        """ grabs the current yaw angle and stores it as the zero angle 
            - which is used to correct all measurements for user 
            - interfacing and for blimp stabilization
        """

        # wait until data has been reported
        while not self.sh_reported.buf[0] and self.run[0]: pass

        # protected read
        [_, _, yw] = self.get_euler()

        # protected write
        self.lock_rot0.acquire()
        self.rot0[2] = yw
        self.lock_rot0.release()
