import multiprocessing as mp
import numpy as np
import os
import signal
import struct
import time
import traceback

from multiprocessing import shared_memory as sm
from numpy import pi
from piStreaming.rcv_pi02 import MultiRcv
from pyBlimp.mixer import *
from pyBlimp.pid import PID
from pyBlimp.utils import *
from scipy.spatial.transform import Rotation as R


def handle_controller(ser, cfg, locks, names, extras, logger):
    """ private controller handler spawned by a parent Blimp object
        - this handle should NOT be used by an application
        - only spawned and controlled by a parent Blimp object    
    """
    C = _Controller(ser, cfg, locks, names, extras, logger)
    C.run()

class _Controller:
    """ private controller that runs on a spawned process to control the blimp
        - this controller should NOT be used by an application
        - only spawned and controlled by a parent Blimp object    
    """

    def __init__(self, ser, cfg, locks, names, extras, logger=False):
        im_sz = (cfg['im_rows'], cfg['im_cols'], 3)
        self.id_num = cfg['id_num']
        self.positive_only = cfg['positive_only']
        self.logger = logger
        self.hz = 100.

        # store serManager interface
        self.lock_ser = ser[0]
        self.cmd_ser = sm.SharedMemory(ser[1])
        
        # store locks and setup shared memory
        self.lock_x = locks[0]
        self.sh_x = sm.SharedMemory(names[0])        
        self.sh_x_stamp = sm.SharedMemory(names[1])
        self.x = np.ndarray(10, dtype=np.double, buffer=self.sh_x.buf)
        self.x_stamp = np.ndarray(1, np.double, buffer=self.sh_x_stamp.buf)

        self.lock_I = locks[1]
        self.sh_img = sm.SharedMemory(names[2])        
        self.sh_img_stamp = sm.SharedMemory(names[3])
        self.img = np.ndarray(im_sz, dtype=np.uint8, buffer=self.sh_img.buf)
        self.img_stamp = np.ndarray(1, np.double, buffer=self.sh_img_stamp.buf)

        self.lock_des = locks[2]
        self.sh_des = sm.SharedMemory(names[4])
        self.des = np.ndarray(4, dtype=np.double, buffer=self.sh_des.buf)

        self.lock_rot0 = locks[3]
        self.sh_rot0 = sm.SharedMemory(names[5])
        self.rot0 = np.ndarray(3, dtype=np.double, buffer=self.sh_rot0.buf)

        self.lock_cmd = locks[4]
        self.sh_man = sm.SharedMemory(names[6])
        self.man = np.ndarray(1, dtype=np.bool_, buffer=self.sh_man.buf)
        self.sh_cmd = sm.SharedMemory(names[7])
        self.cmd = np.ndarray(4, dtype=np.double, buffer=self.sh_cmd.buf)        

        self.sh_run = sm.SharedMemory(names[8])
        self.running = np.ndarray(1, dtype=np.bool_, buffer=self.sh_run.buf)
        self.sh_reported = sm.SharedMemory(names[9])

        # setup extra shared memory
        self.sh_extra = dict()
        self.extra = dict()
        self.lock_extra = dict()

        for e in extras:
            name = e[0]
            sh_extra = sm.SharedMemory(e[1])
            extra = np.ndarray(e[2], dtype=e[3], buffer=sh_extra.buf)
            self.sh_extra[name] = sh_extra
            self.extra[name] = extra
            self.lock_extra[name] = e[4]

        # setup sensing tools (and sleep to read some data)
        self.pi = MultiRcv(cfg)

        # construct lowest-level PID controllers
        k_r  = np.array(cfg['k_r'])
        k_p  = np.array(cfg['k_p'])
        k_yw = np.array(cfg['k_yw'])
        k_pz = np.array(cfg['k_pz'])
        self.pid_roll  = PID(k_r,  angle=True)
        self.pid_pitch = PID(k_p,  angle=True)
        self.pid_yaw   = PID(k_yw, angle=True)
        self.pid_pz    = PID(k_pz, windup=0.5)

        # setup logger
        if self.logger:
            self.dh = []
            self.uh = []
            self.xh = []
            self.Ih = []
            self.It = []
            self.th = []

            self.eh = dict()
            for e in extras:
                self.eh[e[0]] = []

        # setup control-c handler
        signal.signal(signal.SIGINT, self.handler)
        self.i = 0
       
    def handler(self, signum, frame):
        # - ctrl-c handler to start process cleanup
        self.running[0] = False

        # set to zeros to keep from flying away
        self.des[:4] = 0.

    # main loop
    def run(self):
        # main control loop that runs at a desired frequency
        # - executes the angle controller at a high rate
        # - handles the associated shutdown

        # run this loop continuously at desired rate
        while self.running[0]:
            self.poll()
            self.step()
            time.sleep(1./self.hz)

        # shutdown the raspberry pi
        self.pi.shutdown()

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
        self.cmd_ser.close()

        for key in self.sh_extra:
            self.sh_extra[key].close()

        # return if logger flag not active   
        if not self.logger: return
        
        # save data to directory
        tsave = str(np.datetime64('now')).replace(":", "_")

        dsave = "targets.npy"
        usave = "inputs.npy"
        xsave = "states.npy"
        Isave = "images.npy"
        Itsave = "image_stamps.npy"
        Ssave = "stamps.npy"

        # concatenate to 2D arrays
        dh = np.stack(self.dh).T
        uh = np.stack(self.uh).T
        xh = np.stack(self.xh).T
        Ih = np.stack(self.Ih)
        Iht = np.stack(self.It)
        sh = np.stack(self.th)

        # check if save_dir exists first
        save_dir = "data"
        save_unique_dir = os.path.join(save_dir, "blimp"+str(self.id_num))
        spec_dir = os.path.join(save_unique_dir, tsave)
        if not os.path.isdir(save_dir):
            os.mkdir(save_dir)

        # check if unique_dir exists first
        if not os.path.isdir(save_unique_dir):
            os.mkdir(save_unique_dir)

        # check if specific_dir exists first
        if not os.path.isdir(spec_dir):
            os.mkdir(spec_dir)

        # save blimp data
        np.save(os.path.join(spec_dir, dsave), dh)
        np.save(os.path.join(spec_dir, usave), uh)
        np.save(os.path.join(spec_dir, xsave), xh)
        np.save(os.path.join(spec_dir, Isave), Ih)
        np.save(os.path.join(spec_dir, Itsave), Iht)
        np.save(os.path.join(spec_dir, Ssave), sh)

        for key in self.eh:
            esave = key+".npy"
            esave = esave.replace("/", "_")
            data = np.stack(self.eh[key]).T
            np.save(os.path.join(spec_dir, esave), data)            

        print("Saved", xh.shape[1], "samples in", spec_dir)
        print("Saved", Ih.shape[0], "images in", spec_dir)

    # interfacing with pi
    def poll(self):
        # get most recent bno and distance data
        stamp, bno = self.pi.get_bno()
        _, z = self.pi.get_dis()

        if bno[0][3] != -1.:
            # protected read
            self.lock_rot0.acquire()
            rot0 = self.rot0.copy()
            self.lock_rot0.release()
            eul = quat2euler(bno[0])

            # protected write
            self.lock_x.acquire()
            self.x[0]   =  z               # altitude
            self.x[1:4] =  wrap(eul-rot0)  # euler
            self.x[4]   =  bno[1][0]       # vroll
            self.x[5]   = -bno[1][1]       # vpitch
            self.x[6]   = -bno[1][2]       # vyaw
            self.x[7]   =  bno[2][0]       # xacc
            self.x[8]   = -bno[2][1]       # yacc 
            self.x[9]   = -bno[2][2]       # zacc
            self.x_stamp[0] = stamp
            self.lock_x.release()

            self.sh_reported.buf[0] = True

        # get most recent image
        stamp, I = self.pi.get_image()

        if I is not None:
            # protected write
            self.lock_I.acquire()
            self.img[:,:,:] = I
            self.img_stamp[0] = stamp
            self.lock_I.release()

    def step(self):
        # input to write
        u = np.zeros(6)

        # read necessary data
        x_ = self.x.copy()

        self.lock_des.acquire()
        des = self.des.copy()
        self.lock_des.release()

        # manual flight control
        if self.man[0]:
            self.lock_cmd.acquire()
            u[0] = self.cmd[0]
            u[1] = self.cmd[1]
            u[2] = self.cmd[2]
            u[5] = self.cmd[3]
            self.lock_cmd.release()
            
        # autopilot flight control                    
        else:   
            # check if data has arrived
            if not self.sh_reported.buf[0]: return

            # get altitude, roll, pitch, yaw and velocities
            z, eul, veul = x_[0], x_[1:4], x_[4:7]

            # - inputs
            r_  = np.array([eul[0], veul[0]])
            p_  = np.array([eul[1], veul[1]])
            yw_ = np.array([eul[2], veul[2]])

            des_r_ = np.array([des[0], 0.])
            des_p_ = np.array([des[1], 0.])
            des_yw_ = np.array([des[2], 0.])
            des_z_ = des[3]

            u[0] = self.pid_pitch.input(p_, des_p_, given_velocity=True)
            u[1] = self.pid_roll.input(r_, des_r_, given_velocity=True)
            u[2] = -self.pid_pz.input(z, des_z_)
            u[5] = -self.pid_yaw.input(yw_, des_yw_, given_velocity=True)

            if self.positive_only:
                u[2] = min(u[2], 0.)

        # prioritize vertical thrust
        updown_abs = u[2]
        if abs(updown_abs) > 0.1:
            u[2] *= 0.1

        # clip extraneous input
        u[2] = updown_abs
        if self.positive_only: u[2] = min(u[2], 0)
        u = np.clip(u, -0.8, 0.8)

        # mix commands to get motor inputs
        duty_cycles = mix_inputs(u)

        # add unique ID to end
        msg = convertCMD(duty_cycles, self.id_num)

        msgb = b''
        for val in msg:
            msgb += struct.pack('!B', int(val))

        # try to write a command unless already occupied
        self.lock_ser.acquire()
        self.cmd_ser.buf[:len(msgb)] = msgb
        self.lock_ser.release()

        # log data (if requested)
        if self.logger:
            self.lock_I.acquire()
            I = self.img.copy()
            self.lock_I.release()

            self.dh.append(des.copy())
            self.uh.append(u.copy())
            self.xh.append(x_.copy())
            self.th.append(self.x_stamp[0].copy())

            if len(self.Ih) == 0 or np.sum(I-self.Ih[-1]) != 0:
                self.Ih.append(I.copy())
                self.It.append(self.x_stamp[0].copy())

            for key in self.extra:
                self.lock_extra[key].acquire()
                extra = self.extra[key].copy()
                self.lock_extra[key].release()
                self.eh[key].append(extra)
