import matplotlib.pyplot as plt
import numpy as np
import signal
import time

from pyBlimp.blimp import BlimpManager
from pyBlimp.utils import *
from utils.js_utils import JoyStick_helper

if __name__ == "__main__":
    # setup exit on ctrl-c
    running = True
    def exit_handle(signum, frame):
        global running
        running = False
        
    signal.signal(signal.SIGINT, exit_handle)

    # load desired configs
    cfg_paths = ["configs/config2.yaml"]
    cfg = read_config(cfg_paths)

    # build the blimp object
    b = BlimpManager(cfg, "/dev/ttyUSB0", logger=False)

    # setup the joystick reader
    js = JoyStick_helper()

    # show the FPV
    fig, axes = plt.subplots(1,1)
    I = b.get_image(0)
    h = axes.imshow(I)
    axes.set_xticks([])
    axes.set_yticks([])
    axes.set_title("FPV")

    # manual/assisted mode flag
    manual_mode = True
    des = np.zeros(4)
    des[3] = 0
    
    while running and b.get_running(0):
        # handle the joystick
        ax, ybutton = js.get_state()

        # swap between manual and assisted modes
        if ybutton:
            if not manual_mode:
                b.zero_z_rot(0)
                des = np.zeros(4)
                #des[3] = 1.5
                
            manual_mode = not manual_mode
        
        if manual_mode:
            cmd = np.zeros(4)
            
            # set inputs
            cmd[0] = ax[1]
            cmd[1] = ax[0]
            cmd[2] = -ax[3]
            cmd[3] = ax[2]
            b.set_cmd(cmd, 0)

        else:
            # decide inputs
            des[0] =  0.5*ax[0]
            des[1] =  0.5*ax[1]
            des[2] = wrap(des[2]-0.05*ax[2])
            des[3] = np.clip(des[3]+0.05*ax[3], 0.0, 2.5)
            b.set_des(des, 0)

        # show the feed
        I = b.get_image(0)
        h.set_data(I)
        plt.draw(); plt.pause(0.0001)

        # break if the figure is closed
        if not plt.fignum_exists(fig.number): running = False

    b.shutdown()

